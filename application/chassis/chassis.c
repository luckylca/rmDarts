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

// static referee_info_t* referee_data; // 用于获取裁判系统的数据
extern referee_info_t* referee_info;
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

int speed = 0;

int flag_1=0;
int flag_2=0;

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
#define CHASSIS_3508_MAX_SPEED_CMD 15000.0f//通过这个控制整体速度，原来是 20000.0f
#define CHASSIS_3508_MIN_SPEED_CMD 9000.0f
#define CHASSIS_3508_MAX_LOAD_SPEED_CMD 3500.0f
#define CHASSIS_3508_HOLD_SPEED_CMD 4000.0f
#define CHASSIS_3508_POS2SPEED_KP 4.5f
#define CHASSIS_3508_CROSS_DEADBAND 1200.0f
#define CHASSIS_3508_SLOW_ZONE 2000.0f
#define CHASSIS_3508_HOLD_DEADBAND 400.0f
#define CHASSIS_3508_HOLD_RELEASE_DEADBAND 500.0f
#define CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD 1800.0f   //换弹期间保持速度大小，要换
#define CHASSIS_3508_LOAD_SPEED_CMD 3200.0f          //蓄力期间保持速度大小

#define AUTO_ENTRY_LOAD_ACCEPT_ERR 400.0f
#define AUTO_ENTRY_LOAD_ACCEPT_ERR_SECOND 2000.0f
#define AUTO_ENTRY_LOAD_TIMEOUT_MS 3500.0f


static float Clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}


//保持当前状态
static void ChassisHoldAtLoadWithBias(float L_target, float R_target, float L_force, float R_force)
{
    float err_l = L_target - motor_lf->measure.total_angle;
    float err_r = R_target - motor_rf->measure.total_angle;

    float hold_ref_l = (err_l >= 0.0f) ? -L_force : L_force;
    float hold_ref_r = (err_r >= 0.0f) ? R_force : -R_force;

    DJIMotorSetRef(motor_lf, hold_ref_l);
    DJIMotorSetRef(motor_rf, hold_ref_r);
}

extern int allowed_max_step;

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
static int tmpa=0;
int i = 0;
int test = 0;
int flag_reached =0;
static float auto_reload_wait_start_ms = 0.0f;
static float auto_load_wait_start_ms =0.0f;
void ChassisTask()
{

    #ifdef ONE_BOARD
        SubGetMessage(chassis_sub, &chassis_cmd_recv);
    #endif
    #ifdef CHASSIS_BOARD
        chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
    #endif // CHASSIS_BOARD

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
            DJIMotorSetRef(motor_lf, chassis_cmd_recv.v1);
            DJIMotorSetRef(motor_rf, chassis_cmd_recv.v1);
            break;
        case AUTO_MODE:
            DJIMotorEnable(motor_lf);
            DJIMotorEnable(motor_rf);
            DJIMotorOuterLoop(motor_lf, SPEED_LOOP);
            DJIMotorOuterLoop(motor_rf, SPEED_LOOP);
            // DJIMotorOuterLoop(motor_lf, ANGLE_LOOP);
            // DJIMotorOuterLoop(motor_rf, ANGLE_LOOP);
            int cur = DartSys.currentStep;
            if (tmpa==1)
                return;
            if (DartSys.currentStep > allowed_max_step) {
                break;
            }
            switch (cur)
            {
                case 0:
                    switch(i)
                    {   
                        case 0:
                        {
                            i=1;
                        }
                        case 1:
                        {
                            //此时位于目标蓄力位置，并做停顿
                            
                            float err_l_load = LF_CHASSIS_3508_LOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r_load = RF_CHASSIS_3508_LOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l_load = fabsf(err_l_load);
                            float abs_err_r_load = fabsf(err_r_load);
                            bool load_reached = ((abs_err_l_load < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r_load < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l_load < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r_load < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_load_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                if ((DWT_GetTimeline_ms() - auto_load_wait_start_ms) >= 500.0f)
                                {
                                    i = 2;
                                    auto_load_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l_load >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r_load >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                                DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                                // 记录到位时间，给机构预留稳定时间，避免立刻反向造成冲击
                                if (auto_load_wait_start_ms <= 0.0f)
                                {
                                    auto_load_wait_start_ms = DWT_GetTimeline_ms();
                                }
                                flag_1=3;
                            }
                            else if ((abs_err_l_load < CHASSIS_3508_SLOW_ZONE) || (abs_err_r_load < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);

                                speed=slow_speed_l;
                                DJIMotorSetRef(motor_lf, (err_l_load >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r_load >= 0.0f) ? slow_speed_r : -slow_speed_r);
                                flag_1=2;
                            }
                            else
                            {
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                                flag_1=1;
                            }

                            // 这里写到位的判断：双侧都进入到位死区后，先停止并保持在当前位置

                            break;
                        }

                        case 2:
                        {
                            // ---------------------------
                            // case 0 / i = 1：再往上走到 REBOUND 位置
                            // 同样按你的框架：匀速 -> 减速 -> 到位保持 -> 结束一次
                            // ---------------------------
                            float err_l = LF_CHASSIS_3508_REBOUND_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RF_CHASSIS_3508_REBOUND_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);

                            // 先快速并且匀速往上走
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            DJIMotorSetRef(motor_lf, ref_l);
                            DJIMotorSetRef(motor_rf, ref_r);

                            // 这里写进入减速区的判断
                            if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }

                            // 这里写到位的判断
                            if ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND))
                            {
                                DJIMotorSetRef(motor_lf, 0);
                                DJIMotorSetRef(motor_rf, 0);
                                test = 1;
                                DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                                DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                case 1:
                    switch(i)
                    {   
                        case 0:
                        {
                            // ---------------------------
                            // case 0 / i = 0：先往下走到 LOAD 位置
                            // 按你的框架执行：
                            // 1) 先快速并匀速移动
                            // 2) 进入减速区后自动降速
                            // 3) 到位后保持（给 0 参考）
                            // 4) 满足下一步条件后切到 i=1
                            // ---------------------------
                            float err_l = LF_1_RELOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RE_1_RELOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);
                            bool load_reached = ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_reload_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_1_RELOAD_ANGLE, RE_1_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                if ((flag_reached==1) && ((DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 200.0f))
                                {
                                    flag_reached=0;
                                    i = 1;
                                    auto_reload_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_1_RELOAD_ANGLE, RE_1_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_BOTTOM_REACHED);
                                DART_SET_BIT(cur, FLAG_R_BOTTOM_REACHED);
                                // 记录到位时间，给机构预留稳定时间，避免立刻反向造成冲击
                                flag_2= 3;
                            }
                            else if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);

                                speed=slow_speed_l;
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                                flag_2=2;
                            }
                            else{
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                                flag_2=1;
                            }


                            if(DART_CHECK_MASK(1,MASK_READY_TO_CHASSIS))
                            {
                                flag_reached = 1; 
                                auto_reload_wait_start_ms = DWT_GetTimeline_ms();
                            }
                            // 这里写进行下一步动作的判断：到位并稳定 1s 后，切换到 i=1（上行）
                            // if ((auto_reload_wait_start_ms > 0.0f) &&
                            //     ((DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 1000.0f))
                            // {
                            //     i = 1;
                            //     auto_reload_wait_start_ms = 0.0f;
                            // }//这里或许可以注释掉
                            break;
                        }
                        case 1:
                        {
                            //此时位于目标蓄力位置，并做停顿
                            
                            float err_l_load = LF_CHASSIS_3508_LOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r_load = RF_CHASSIS_3508_LOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l_load = fabsf(err_l_load);
                            float abs_err_r_load = fabsf(err_r_load);
                            bool load_reached = ((abs_err_l_load < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r_load < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l_load < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r_load < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_load_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                if ((DWT_GetTimeline_ms() - auto_load_wait_start_ms) >= 100.0f)
                                {
                                    i = 2;
                                    auto_load_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l_load >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r_load >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                    

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                                DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                                // 记录到位时间，给机构预留稳定时间，避免立刻反向造成冲击
                                if (auto_load_wait_start_ms <= 0.0f)
                                {
                                    auto_load_wait_start_ms = DWT_GetTimeline_ms();
                                }
                            }
                            else if ((abs_err_l_load < CHASSIS_3508_SLOW_ZONE) || (abs_err_r_load < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l_load >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r_load >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }
                            else{
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                            }
                        

                            break;
                        }

                        case 2:
                        {
                            // ---------------------------
                            // case 0 / i = 1：再往上走到 REBOUND 位置
                            // 同样按你的框架：匀速 -> 减速 -> 到位保持 -> 结束一次
                            // ---------------------------
                            float err_l = LF_CHASSIS_3508_REBOUND_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RF_CHASSIS_3508_REBOUND_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);

                            // 先快速并且匀速往上走
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            DJIMotorSetRef(motor_lf, ref_l);
                            DJIMotorSetRef(motor_rf, ref_r);

                            // 这里写进入减速区的判断
                            if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }

                            // 这里写到位的判断
                            if ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND))
                            {
                                DJIMotorSetRef(motor_lf, 0);
                                DJIMotorSetRef(motor_rf, 0);
                                test = 1;
                                DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                                DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                case 2:

                    switch(i)
                    {   
                        case 0:
                        {
                            // ---------------------------
                            // case 0 / i = 0：先往下走到 LOAD 位置
                            // 按你的框架执行：
                            // 1) 先快速并匀速移动
                            // 2) 进入减速区后自动降速
                            // 3) 到位后保持（给 0 参考）
                            // 4) 满足下一步条件后切到 i=1
                            // ---------------------------
                            float err_l = LF_2_RELOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RE_2_RELOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);
                            bool load_reached = ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_reload_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_2_RELOAD_ANGLE, RE_2_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                if (flag_reached==1 && (DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 200.0f)
                                {
                                    flag_reached=0;
                                    i = 1;
                                    auto_reload_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_2_RELOAD_ANGLE, RE_2_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_BOTTOM_REACHED);
                                DART_SET_BIT(cur, FLAG_R_BOTTOM_REACHED);
                            }
                            else if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                speed = slow_speed_l;
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }
                            else
                            {
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                            }

                            // // 这里写到位的判断：双侧都进入到位死区后，先停止并保持在当前位置
                            
                            // 这里写进行下一步动作的判断：到位并稳定 1s 后，切换到 i=1（上行）
                            // if ((auto_reload_wait_start_ms > 0.0f) &&
                            //     ((DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 1000.0f))
                            // {
                            //     i = 1;
                            //     auto_reload_wait_start_ms = 0.0f;
                            // }//这里或许可以注释掉
                            if(DART_CHECK_MASK(2,MASK_READY_TO_CHASSIS))
                            {
                                flag_reached = 1; 
                                auto_reload_wait_start_ms = DWT_GetTimeline_ms();
                            }
                            break;
                        }
                        case 1:
                        {
                            //此时位于目标蓄力位置，并做停顿
                            
                            float err_l_load = LF_CHASSIS_3508_LOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r_load = RF_CHASSIS_3508_LOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l_load = fabsf(err_l_load);
                            float abs_err_r_load = fabsf(err_r_load);
                            bool load_reached = ((abs_err_l_load < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r_load < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l_load < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r_load < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_load_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                if ((DWT_GetTimeline_ms() - auto_load_wait_start_ms) >= 100.0f)
                                {
                                    i = 2;
                                    auto_load_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l_load >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r_load >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                                DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                                // 记录到位时间，给机构预留稳定时间，避免立刻反向造成冲击
                                if (auto_load_wait_start_ms <= 0.0f)
                                {
                                    auto_load_wait_start_ms = DWT_GetTimeline_ms();
                                }
                            }
                            else if ((abs_err_l_load < CHASSIS_3508_SLOW_ZONE) || (abs_err_r_load < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l_load >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r_load >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }
                            else{
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                            }
                            

                            break;
                        }

                        case 2:
                        {
                            // ---------------------------
                            // case 0 / i = 1：再往上走到 REBOUND 位置
                            // 同样按你的框架：匀速 -> 减速 -> 到位保持 -> 结束一次
                            // ---------------------------
                            float err_l = LF_CHASSIS_3508_REBOUND_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RF_CHASSIS_3508_REBOUND_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);

                            // 先快速并且匀速往上走
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            DJIMotorSetRef(motor_lf, ref_l);
                            DJIMotorSetRef(motor_rf, ref_r);

                            // 这里写进入减速区的判断
                            if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }

                            // 这里写到位的判断
                            if ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND))
                            {
                                DJIMotorSetRef(motor_lf, 0);
                                DJIMotorSetRef(motor_rf, 0);
                                test = 1;
                                DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                                DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                case 3:
                    switch(i)
                    {   
                        case 0:
                        {
                            // ---------------------------
                            // case 0 / i = 0：先往下走到 LOAD 位置
                            // 按你的框架执行：
                            // 1) 先快速并匀速移动
                            // 2) 进入减速区后自动降速
                            // 3) 到位后保持（给 0 参考）
                            // 4) 满足下一步条件后切到 i=1
                            // ---------------------------
                            float err_l = LF_3_RELOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RE_3_RELOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);
                            bool load_reached = ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_reload_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_3_RELOAD_ANGLE, RE_3_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                if (flag_reached==1 && (DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 200.0f)
                                {
                                    flag_reached=0;
                                    i = 1;
                                    auto_reload_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;

                            // 这里写进入减速区的判断：任一侧进入慢速区，就切换为减速参考
                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_3_RELOAD_ANGLE, RE_3_RELOAD_ANGLE,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD,CHASSIS_3508_BOTTOM_HOLD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_BOTTOM_REACHED);
                                DART_SET_BIT(cur, FLAG_R_BOTTOM_REACHED);
                            }
                            else if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_MAX_LOAD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                speed = slow_speed_l;
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }
                            else
                            {
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                            }

                            // 这里写到位的判断：双侧都进入到位死区后，先停止并保持在当前位置
                            // 这里写进行下一步动作的判断：到位并稳定 1s 后，切换到 i=1（上行）
                            // if ((auto_reload_wait_start_ms > 0.0f) &&
                            //     ((DWT_GetTimeline_ms() - auto_reload_wait_start_ms) >= 1000.0f))
                            // {
                            //     i = 1;
                            //     auto_reload_wait_start_ms = 0.0f;
                            // }//这里或许可以注释掉
                            if(DART_CHECK_MASK(3,MASK_READY_TO_CHASSIS))
                            {
                                auto_reload_wait_start_ms = DWT_GetTimeline_ms();
                                flag_reached = 1;
                            }
                            
                            break;
                        }
                        case 1:
                        {
                            //此时位于目标蓄力位置，并做停顿
                            
                            float err_l_load = LF_CHASSIS_3508_LOAD_ANGLE - motor_lf->measure.total_angle;
                            float err_r_load = RF_CHASSIS_3508_LOAD_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l_load = fabsf(err_l_load);
                            float abs_err_r_load = fabsf(err_r_load);
                            bool load_reached = ((abs_err_l_load < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r_load < CHASSIS_3508_HOLD_DEADBAND)) ||
                                                ((abs_err_l_load < AUTO_ENTRY_LOAD_ACCEPT_ERR) && (abs_err_r_load < AUTO_ENTRY_LOAD_ACCEPT_ERR));

                            if (auto_load_wait_start_ms > 0.0f)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                if ((DWT_GetTimeline_ms() - auto_load_wait_start_ms) >= 100.0f)
                                {
                                    i = 2;
                                    auto_load_wait_start_ms = 0.0f;
                                }
                                break;
                            }

                            // 先快速并且匀速往下走：远离目标时直接给最大速度
                            float ref_l = (err_l_load >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r_load >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;

                            if (load_reached)
                            {
                                ChassisHoldAtLoadWithBias(LF_CHASSIS_3508_LOAD_ANGLE,RF_CHASSIS_3508_LOAD_ANGLE,CHASSIS_3508_LOAD_SPEED_CMD,CHASSIS_3508_LOAD_SPEED_CMD);
                                DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                                DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                                // 记录到位时间，给机构预留稳定时间，避免立刻反向造成冲击
                                if (auto_load_wait_start_ms <= 0.0f)
                                {
                                    auto_load_wait_start_ms = DWT_GetTimeline_ms();
                                }
                            }
                            else if ((abs_err_l_load < CHASSIS_3508_SLOW_ZONE) || (abs_err_r_load < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r_load * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l_load >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r_load >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }
                            else
                            {
                                DJIMotorSetRef(motor_lf, ref_l);
                                DJIMotorSetRef(motor_rf, ref_r);
                            }

                            break;
                        }

                        case 2:
                        {
                            // ---------------------------
                            // case 0 / i = 1：再往上走到 REBOUND 位置
                            // 同样按你的框架：匀速 -> 减速 -> 到位保持 -> 结束一次
                            // ---------------------------
                            float err_l = LF_CHASSIS_3508_REBOUND_ANGLE - motor_lf->measure.total_angle;
                            float err_r = RF_CHASSIS_3508_REBOUND_ANGLE - motor_rf->measure.total_angle;
                            float abs_err_l = fabsf(err_l);
                            float abs_err_r = fabsf(err_r);

                            // 先快速并且匀速往上走
                            float ref_l = (err_l >= 0.0f) ? -CHASSIS_3508_MAX_SPEED_CMD : CHASSIS_3508_MAX_SPEED_CMD;
                            float ref_r = (err_r >= 0.0f) ? CHASSIS_3508_MAX_SPEED_CMD : -CHASSIS_3508_MAX_SPEED_CMD;
                            DJIMotorSetRef(motor_lf, ref_l);
                            DJIMotorSetRef(motor_rf, ref_r);

                            // 这里写进入减速区的判断
                            if ((abs_err_l < CHASSIS_3508_SLOW_ZONE) || (abs_err_r < CHASSIS_3508_SLOW_ZONE))
                            {
                                float slow_speed_l = Clampf(abs_err_l * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                float slow_speed_r = Clampf(abs_err_r * CHASSIS_3508_POS2SPEED_KP,
                                                            CHASSIS_3508_HOLD_SPEED_CMD,
                                                            CHASSIS_3508_MIN_SPEED_CMD);
                                DJIMotorSetRef(motor_lf, (err_l >= 0.0f) ? -slow_speed_l : slow_speed_l);
                                DJIMotorSetRef(motor_rf, (err_r >= 0.0f) ? slow_speed_r : -slow_speed_r);
                            }

                            // 这里写到位的判断
                            if ((abs_err_l < CHASSIS_3508_HOLD_DEADBAND) && (abs_err_r < CHASSIS_3508_HOLD_DEADBAND))
                            {
                                DJIMotorSetRef(motor_lf, 0);
                                DJIMotorSetRef(motor_rf, 0);
                                test = 1;
                                DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                                DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                    break;
        default:
            break;
        }
            
           /*
            switch (cur) {
                case 0:
                    switch(i)
                    {   
                        case 0:
                        {
                            i=1;
                            break;
                        }
                        case 1:
                        {
                            DJIMotorSetRef(motor_lf, RF_CHASSIS_3508_LOAD_ANGLE);
                            DJIMotorSetRef(motor_rf, RF_CHASSIS_3508_LOAD_ANGLE);
                            if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, LF_CHASSIS_3508_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND)&&CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, RF_CHASSIS_3508_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND))
                            {
                                DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                                DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                                i=2;
                            }
                        }
                        case 2:
                        {
                            DJIMotorSetRef(motor_lf, LF_CHASSIS_3508_REBOUND_ANGLE);
                            DJIMotorSetRef(motor_rf, RF_CHASSIS_3508_REBOUND_ANGLE);
                            if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, LF_CHASSIS_3508_REBOUND_ANGLE, MOTOR_ANGLE_DEADBAND)&&CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, RF_CHASSIS_3508_REBOUND_ANGLE, MOTOR_ANGLE_DEADBAND))
                            {
                                DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                                DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                                i=0;
                            }
                        }
                        default:
                            break;
                    }
                case 1:
                    switch (i)
                    {
                    case 0:
                        DJIMotorSetRef(motor_lf,RE_1_RELOAD_ANGLE);
                        DJIMotorSetRef(motor_rf,RE_1_RELOAD_ANGLE);
                        if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, RE_1_RELOAD_ANGLE, MOTOR_ANGLE_DEADBAND)&&CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, RE_1_RELOAD_ANGLE, MOTOR_ANGLE_DEADBAND))
                        {
                            DART_SET_BIT(cur, FLAG_L_BOTTOM_REACHED);
                            DART_SET_BIT(cur, FLAG_R_BOTTOM_REACHED);
                            if(DART_CHECK_MASK(1,MASK_READY_TO_CHASSIS))
                                i=1;
                        }
                        break;
                    case 1:
                        DJIMotorSetRef(motor_lf, RF_CHASSIS_3508_LOAD_ANGLE);
                        DJIMotorSetRef(motor_rf, RF_CHASSIS_3508_LOAD_ANGLE);
                        if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, LF_CHASSIS_3508_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND)&&CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, RF_CHASSIS_3508_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND))
                        {
                            DART_SET_BIT(cur, FLAG_L_CHARGE_REACHED);
                            DART_SET_BIT(cur, FLAG_R_CHARGE_REACHED);
                            i=2;
                        }
                        break;
                    case 2:
                        DJIMotorSetRef(motor_lf, LF_CHASSIS_3508_REBOUND_ANGLE);
                        DJIMotorSetRef(motor_rf, RF_CHASSIS_3508_REBOUND_ANGLE);
                        if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, LF_CHASSIS_3508_REBOUND_ANGLE, MOTOR_ANGLE_DEADBAND)&&CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, RF_CHASSIS_3508_REBOUND_ANGLE, MOTOR_ANGLE_DEADBAND))
                        {
                            DART_SET_BIT(cur, FLAG_L_REBOUND_REACHED);
                            DART_SET_BIT(cur, FLAG_R_REBOUND_REACHED);
                            i=0;
                        }
                        break;
                    default:
                        break;
                    }




            }
            */
    }

    #ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    #endif
    #ifdef CHASSIS_BOARD
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
    #endif // CHASSIS_BOARD
}
