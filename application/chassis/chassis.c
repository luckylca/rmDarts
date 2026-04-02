/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 负责接收robot_cmd的控制命令并根据命令，进行蓄力电机的控制
 *        
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "status.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <stdbool.h>
#include <stdint.h>
#include "at24c02.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static DJIMotorInstance *motor_lf, *motor_rf; // 两边的蓄力电机

// 位置同步PID
static PIDInstance sync_pid;
// 电流同步PID
static PIDInstance current_sync_pid;
static float sync_out = 0;

// 左右电机上电角度
static float original_angle_left = 0;
static float original_angle_right = 0;

// 读取上电角度标志位
int read_original_3508_angle = 1;
// 与上电角度的误差
float err_of_original_angle = 0;

static float dt = 0;
/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_v1, chassis_v_;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

// 拉力传感器数据
extern double F_data;
// 3508到位标志位
int flag_3508_ready = 0;
extern int flag_arm_sucess;
// 完成机械臂动作

// 等待装载延时标志位
int flag_wait_dart_load_delay = 0; 
// 3508归位的标志位
int flag_3508_back = 0;

//3508到最末尾标志位
int flag_3508_max=0;

//放镖完毕标志位
int flag_loadok = 0;

extern bool flag_2006_target_ready;
extern bool flag_2006_back;
extern goal_of_dart goal;
extern int key;

//固定位置力大小
#define FOCE_3508_STAY1 58
#define FOCE_3508_STAY2 62
#define FOCE_3508_MAX 68
//转动速度
#define SPEED_3508 -4000

// 速度环位置控制参数：远距离高速，近距离自动降速
#define CHASSIS_3508_MAX_SPEED_CMD 20000.0f
#define CHASSIS_3508_MIN_SPEED_CMD 9000.0f
#define CHASSIS_3508_HOLD_SPEED_CMD 3500.0f
#define CHASSIS_3508_POS2SPEED_KP 0.75f
#define CHASSIS_3508_CROSS_DEADBAND 1200.0f
#define CHASSIS_3508_SLOW_ZONE 650.0f
#define CHASSIS_3508_HOLD_DEADBAND 220.0f
#define CHASSIS_3508_HOLD_RELEASE_DEADBAND 500.0f

#define AUTO_ENTRY_LOAD_ACCEPT_ERR 1200.0f
#define AUTO_ENTRY_LOAD_TIMEOUT_MS 3500.0f


float v = -4000;  //转动速度

static float prev_err_test_l = 0.0f;
static float prev_err_test_r = 0.0f;
static float prev_err_load_l[4] = {0};
static float prev_err_load_r[4] = {0};
static float prev_err_rebound_l[4] = {0};
static float prev_err_rebound_r[4] = {0};

typedef enum {
    AUTO_ENTRY_IDLE = 0,
    AUTO_ENTRY_TO_LOAD,
    AUTO_ENTRY_WAIT_1S,
    AUTO_ENTRY_TO_REBOUND,
    AUTO_ENTRY_DONE,
} AutoEntryState_e;

static chassis_mode_e s_last_chassis_mode = CHASSIS_ZERO_FORCE;
static AutoEntryState_e s_auto_entry_state = AUTO_ENTRY_IDLE;
static float s_auto_wait_start_ms = 0.0f;
static float s_auto_stage_start_ms = 0.0f;

static bool s_hold_l = false;
static bool s_hold_r = false;

static void ChassisDriveResetState(float *prev_l, float *prev_r)
{
    if (prev_l != NULL)
    {
        *prev_l = 0.0f;
    }
    if (prev_r != NULL)
    {
        *prev_r = 0.0f;
    }
    s_hold_l = false;
    s_hold_r = false;
}

static float Clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static float ChassisPosToSpeedRef(float current_angle, float target_angle)
{
    float err = target_angle - current_angle;
    float speed_abs = Clampf(CHASSIS_3508_POS2SPEED_KP * fabsf(err), CHASSIS_3508_MIN_SPEED_CMD, CHASSIS_3508_MAX_SPEED_CMD);
    return (err >= 0.0f) ? speed_abs : -speed_abs;
}

static bool ChassisAngleArrivedWithCross(float current_angle, float target_angle, float deadband, float *prev_err)
{
    float err = target_angle - current_angle;
    bool in_deadband = fabsf(err) < deadband;
    bool crossed = ((*prev_err > 0.0f && err < 0.0f) || (*prev_err < 0.0f && err > 0.0f)) &&
                   (fabsf(err) < CHASSIS_3508_CROSS_DEADBAND);
    *prev_err = err;
    return (in_deadband || crossed);
}

static bool ChassisDriveToTargetSpeedLoop(float target_l, float target_r, float *prev_l, float *prev_r)
{
    float err_l = target_l - motor_lf->measure.total_angle;
    float err_r = target_r - motor_rf->measure.total_angle;
    float err_abs_l = fabsf(err_l);
    float err_abs_r = fabsf(err_r);
    float speed_abs_l;
    float speed_abs_r;

    if (s_hold_l)
    {
        if (err_abs_l > CHASSIS_3508_HOLD_RELEASE_DEADBAND)
        {
            s_hold_l = false;
        }
    }
    else if (err_abs_l <= CHASSIS_3508_HOLD_DEADBAND)
    {
        s_hold_l = true;
    }

    if (err_abs_r <= CHASSIS_3508_HOLD_DEADBAND)
    {
        s_hold_r = true;
    }
    else if (s_hold_r && err_abs_r > CHASSIS_3508_HOLD_RELEASE_DEADBAND)
    {
        s_hold_r = false;
    }

    if (s_hold_l)
    {
        speed_abs_l = 0.0f;
    }
    else if (err_abs_l < CHASSIS_3508_SLOW_ZONE)
    {
        speed_abs_l = Clampf(CHASSIS_3508_POS2SPEED_KP * err_abs_l, CHASSIS_3508_HOLD_SPEED_CMD, CHASSIS_3508_MIN_SPEED_CMD);
    }
    else
    {
        speed_abs_l = Clampf(CHASSIS_3508_POS2SPEED_KP * err_abs_l, CHASSIS_3508_MIN_SPEED_CMD, CHASSIS_3508_MAX_SPEED_CMD);
    }

    if (s_hold_r)
    {
        speed_abs_r = 0.0f;
    }
    else if (err_abs_r < CHASSIS_3508_SLOW_ZONE)
    {
        speed_abs_r = Clampf(CHASSIS_3508_POS2SPEED_KP * err_abs_r, CHASSIS_3508_HOLD_SPEED_CMD, CHASSIS_3508_MIN_SPEED_CMD);
    }
    else
    {
        speed_abs_r = Clampf(CHASSIS_3508_POS2SPEED_KP * err_abs_r, CHASSIS_3508_MIN_SPEED_CMD, CHASSIS_3508_MAX_SPEED_CMD);
    }

    // 机械方向约定：
    // LF: 位置从大到小时速度应为正
    // RF: 与LF相反
    float speed_ref_l = (err_l >= 0.0f) ? -speed_abs_l : speed_abs_l;
    float speed_ref_r = (err_r >= 0.0f) ? speed_abs_r : -speed_abs_r;

    DJIMotorSetRef(motor_lf, speed_ref_l);
    DJIMotorSetRef(motor_rf, speed_ref_r);

    bool left_arrived = ChassisAngleArrivedWithCross(motor_lf->measure.total_angle, target_l, 500.0f, prev_l);
    bool right_arrived = ChassisAngleArrivedWithCross(motor_rf->measure.total_angle, target_r, 500.0f, prev_r);

    if (left_arrived && right_arrived)
    {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        return true;
    }

    return false;
}

static void ChassisRunAutoPullSequence(bool enter_auto, bool leave_auto)
{
    if (enter_auto)
    {
        s_auto_entry_state = AUTO_ENTRY_TO_LOAD;
        s_auto_wait_start_ms = 0.0f;
        s_auto_stage_start_ms = DWT_GetTimeline_ms();
        ChassisDriveResetState(&prev_err_load_l[0], &prev_err_load_r[0]);
    }
    if (leave_auto)
    {
        s_auto_entry_state = AUTO_ENTRY_IDLE;
        s_auto_wait_start_ms = 0.0f;
        s_auto_stage_start_ms = 0.0f;
        ChassisDriveResetState(NULL, NULL);
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        return;
    }

    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorOuterLoop(motor_lf, SPEED_LOOP);
    DJIMotorOuterLoop(motor_rf, SPEED_LOOP);

    if (s_auto_entry_state == AUTO_ENTRY_TO_LOAD)
    {
        bool reached_by_drive = ChassisDriveToTargetSpeedLoop(LF_CHASSIS_3508_LOAD_ANGLE, RF_CHASSIS_3508_LOAD_ANGLE,
                                                               &prev_err_load_l[0], &prev_err_load_r[0]);
        float load_err_l = fabsf(LF_CHASSIS_3508_LOAD_ANGLE - motor_lf->measure.total_angle);
        float load_err_r = fabsf(RF_CHASSIS_3508_LOAD_ANGLE - motor_rf->measure.total_angle);
        bool reached_by_window = (load_err_l < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (load_err_r < AUTO_ENTRY_LOAD_ACCEPT_ERR);
        bool reached_by_timeout = (DWT_GetTimeline_ms() - s_auto_stage_start_ms) >= AUTO_ENTRY_LOAD_TIMEOUT_MS;

        if (reached_by_drive || reached_by_window || reached_by_timeout)
        {
            DJIMotorSetRef(motor_lf, 0);
            DJIMotorSetRef(motor_rf, 0);
            s_auto_wait_start_ms = DWT_GetTimeline_ms();
            s_auto_entry_state = AUTO_ENTRY_WAIT_1S;
        }
    }
    else if (s_auto_entry_state == AUTO_ENTRY_WAIT_1S)
    {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        if ((DWT_GetTimeline_ms() - s_auto_wait_start_ms) >= 1000.0f)
        {
            ChassisDriveResetState(&prev_err_rebound_l[0], &prev_err_rebound_r[0]);
            s_auto_stage_start_ms = DWT_GetTimeline_ms();
            s_auto_entry_state = AUTO_ENTRY_TO_REBOUND;
        }
    }
    else if (s_auto_entry_state == AUTO_ENTRY_TO_REBOUND)
    {
        if (ChassisDriveToTargetSpeedLoop(LF_CHASSIS_3508_REBOUND_ANGLE, RF_CHASSIS_3508_REBOUND_ANGLE,
                                          &prev_err_rebound_l[0], &prev_err_rebound_r[0]))
        {
            s_auto_entry_state = AUTO_ENTRY_DONE;
            DART_SET_BIT(0, FLAG_L_CHARGE_REACHED);
            DART_SET_BIT(0, FLAG_R_CHARGE_REACHED);
            DART_SET_BIT(0, FLAG_L_REBOUND_REACHED);
            DART_SET_BIT(0, FLAG_R_REBOUND_REACHED);
        }
    }
    else
    {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
    }
}

void ChassisInit()
{   
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 20, // 15
                .Ki = 0,   // 0
                .Kd = 0.03, // 0.03
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20000,
            },//角度环 pid 还需要再调
            .speed_PID = {
                .Kp = 4.5, // 4.5
                .Ki = 0,  // 0
                .Kd = 0.0001,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20000,
            },
            .current_PID = {
                .Kp = 1, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;//
    motor_rf = DJIMotorInit(&chassis_motor_config);

    // 初始化同步PID
    PID_Init_Config_s sync_pid_conf = {
        .Kp = 50.0f,
        .Ki = 5.0f,
        .Kd = 0.0f,
        .MaxOut = 20000.0f,
        .DeadBand = 0,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 10000.0f,
    };
    PIDInit(&sync_pid, &sync_pid_conf);

    // 初始化电流同步PID
    PID_Init_Config_s current_sync_pid_conf = {
        .Kp = 0.05f,         // 需根据实际情况调试，建议从小值开始
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 3000.0f,   // 限制电流同步的最大影响，防止干扰位置环
        .DeadBand = 0,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 3000.0f,
    };
    PIDInit(&current_sync_pid, &current_sync_pid_conf);

    // 设置前馈
    motor_lf->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
    motor_rf->motor_settings.feedforward_flag |= CURRENT_FEEDFORWARD;
    motor_lf->motor_controller.current_feedforward_ptr = &sync_out;
    motor_rf->motor_controller.current_feedforward_ptr = &sync_out;

    // Motor_Recoder_Init_Config_s recoder_config;
    // recoder_config.type = DJI_MOTOR;
    // recoder_config.data.dji = motor_lf; 
    // motorRecoderRegister(&recoder_config);
    // recoder_config.data.dji = motor_rf; 
    // motorRecoderRegister(&recoder_config);
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI
    // referee_data = ReTaskInit(&huart1); // 裁判系统初始化
#endif 



#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}


void ChassisTask()
{

    #ifdef ONE_BOARD
        SubGetMessage(chassis_sub, &chassis_cmd_recv);
    #endif
    #ifdef CHASSIS_BOARD
        chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    #endif // CHASSIS_BOARD

    bool enter_auto = (chassis_cmd_recv.chassis_mode == AUTO_MODE) && (s_last_chassis_mode != AUTO_MODE);
    bool leave_auto = (chassis_cmd_recv.chassis_mode != AUTO_MODE) && (s_last_chassis_mode == AUTO_MODE);

    if (leave_auto)
    {
        ChassisRunAutoPullSequence(false, true);
    }

    // 计算同步PID
    // 计算左右电机相对于各自上电初始位置的偏差之和
    // 假设左右对称安装，一正一反运动，理想情况下相对位移之和应为0
    // float sync_error = (motor_lf->measure.total_angle - original_angle_left) + (motor_rf->measure.total_angle - original_angle_right);
    // float sync_val = PIDCalculate(&sync_pid, 0, sync_error);

    // // 计算电流同步PID
    // float current_sync_error = motor_lf->measure.real_current + motor_rf->measure.real_current;
    // float current_sync_val = PIDCalculate(&current_sync_pid, 0, current_sync_error);

    // // 叠加位置和电流的同步输出
    // sync_out = -sync_val - current_sync_val;

    // 根据控制模式设置蓄力状态
    switch (chassis_cmd_recv.chassis_mode)
    {
        case CHASSIS_ZERO_FORCE:
            DJIMotorOuterLoop(motor_lf, SPEED_LOOP);
            DJIMotorOuterLoop(motor_rf, SPEED_LOOP);
            DJIMotorSetRef(motor_lf, 0);
            DJIMotorSetRef(motor_rf, 0);
            break;
        case CHASSIS_TEST: 
            DJIMotorEnable(motor_lf);
            DJIMotorEnable(motor_rf);
            // DJIMotorOuterLoop(motor_lf, ANGLE_LOOP);
            // DJIMotorOuterLoop(motor_rf, ANGLE_LOOP);
            DJIMotorOuterLoop(motor_lf, SPEED_LOOP);
            DJIMotorOuterLoop(motor_rf, SPEED_LOOP);
            // ChassisDriveToTargetSpeedLoop(LF_CHASSIS_3508_LOAD_ANGLE, RF_CHASSIS_3508_LOAD_ANGLE, &prev_err_test_l, &prev_err_test_r);
            DJIMotorSetRef(motor_lf, chassis_cmd_recv.v1);
            DJIMotorSetRef(motor_rf, chassis_cmd_recv.v1);
            break;
        case AUTO_MODE:
            ChassisRunAutoPullSequence(enter_auto, leave_auto);
            break;
        default:
            break;
    }

            s_last_chassis_mode = chassis_cmd_recv.chassis_mode;
    


    #ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    #endif
    #ifdef CHASSIS_BOARD
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
    #endif // CHASSIS_BOARD
}
