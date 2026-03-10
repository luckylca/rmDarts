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


float v = -4000;  //转动速度

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
                .Kp = 1.5, // 4.5
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

    // 正常工作,初始化左右电机初始角度
    if(read_original_3508_angle==1)
    {
        // original_angle_left = motor_lf->measure.total_angle;
        // original_angle_right = motor_rf->measure.total_angle;
        original_angle_left = 0;
        original_angle_right = 0;
        read_original_3508_angle = 0;
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
            DJIMotorOuterLoop(motor_lf, ANGLE_LOOP);
            DJIMotorOuterLoop(motor_rf, ANGLE_LOOP);
            // DJIMotorOuterLoop(motor_lf, SPEED_LOOP);
            // DJIMotorOuterLoop(motor_rf, SPEED_LOOP);
            DJIMotorSetRef(motor_lf, chassis_cmd_recv.v1);
            DJIMotorSetRef(motor_rf, chassis_cmd_recv.v1);
            // DJIMotorSetRef(motor_lf, CHASSIS_3508_LOAD_ANGLE);
            // DJIMotorSetRef(motor_rf, CHASSIS_3508_LOAD_ANGLE);
            break;
        case AUTO_MODE: 
            {
                DJIMotorEnable(motor_lf);
                DJIMotorEnable(motor_rf);
                uint8_t cur = DartSys.currentStep; 
                // 防止数组越界
                if (cur >= 4) return;
            
            // 计算目标角度：基于上电初始位置的相对偏移
            float target_lf_load = original_angle_left - CHASSIS_3508_LOAD_ANGLE;
            float target_rf_load = original_angle_right + CHASSIS_3508_LOAD_ANGLE;
            float target_lf_rebound = original_angle_left - CHASSIS_3508_REBOUND_ANGLE;
            float target_rf_rebound = original_angle_right + CHASSIS_3508_REBOUND_ANGLE;
            float motor_v = 16000; 
            if (cur == 0) {
                //第一发镖
                if (!DART_CHECK_BIT(0, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(0, FLAG_L_CHARGE_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_load, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_load, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(0, FLAG_R_CHARGE_REACHED);
                        DART_SET_BIT(0, FLAG_L_CHARGE_REACHED);
                    }
                }
                if(!DART_CHECK_BIT(0, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(0, FLAG_L_CHARGE_REACHED)) {
                    return;
                }
                if (!DART_CHECK_BIT(0, FLAG_R_REBOUND_REACHED) || !DART_CHECK_BIT(0, FLAG_L_REBOUND_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_rebound, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_rebound, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(0, FLAG_R_REBOUND_REACHED);
                        DART_SET_BIT(0, FLAG_L_REBOUND_REACHED);
                    }
                }
                return;
            }

            // 第二发镖
            if (cur == 1) {
                if (!DART_CHECK_BIT(1, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(1, FLAG_L_CHARGE_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_load, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_load, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(1, FLAG_R_CHARGE_REACHED);
                        DART_SET_BIT(1, FLAG_L_CHARGE_REACHED);
                    }
                }
                if(!DART_CHECK_BIT(1, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(1, FLAG_L_CHARGE_REACHED)) {
                    return;
                }
                if (!DART_CHECK_BIT(1, FLAG_R_REBOUND_REACHED) || !DART_CHECK_BIT(1, FLAG_L_REBOUND_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_rebound, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_rebound, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(1, FLAG_R_REBOUND_REACHED);
                        DART_SET_BIT(1, FLAG_L_REBOUND_REACHED);
                    }
                }
                return;
            }

            // 第三发镖
            if (cur == 2) {
                if (!DART_CHECK_BIT(2, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(2, FLAG_L_CHARGE_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_load, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_load, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(2, FLAG_R_CHARGE_REACHED);
                        DART_SET_BIT(2, FLAG_L_CHARGE_REACHED);
                    }
                }
                if(!DART_CHECK_BIT(2, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(2, FLAG_L_CHARGE_REACHED)) {
                    return;
                }
                if (!DART_CHECK_BIT(2, FLAG_R_REBOUND_REACHED) || !DART_CHECK_BIT(2, FLAG_L_REBOUND_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_rebound, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_rebound, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(2, FLAG_R_REBOUND_REACHED);
                        DART_SET_BIT(2, FLAG_L_REBOUND_REACHED);
                    }
                }
                return;
            }

            // 第四发镖
            if (cur == 3) {
                if (!DART_CHECK_BIT(3, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(3, FLAG_L_CHARGE_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_load, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_load, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(3, FLAG_R_CHARGE_REACHED);
                        DART_SET_BIT(3, FLAG_L_CHARGE_REACHED);
                    }
                }
                if(!DART_CHECK_BIT(3, FLAG_R_CHARGE_REACHED) || !DART_CHECK_BIT(3, FLAG_L_CHARGE_REACHED)) {
                    return;
                }
                if (!DART_CHECK_BIT(3, FLAG_R_REBOUND_REACHED) || !DART_CHECK_BIT(3, FLAG_L_REBOUND_REACHED)) {
                    DJIMotorSetRef(motor_lf, motor_v);
                    DJIMotorSetRef(motor_rf, motor_v);
                    if(CHECK_ANGLE_ARRIVED(motor_lf->measure.total_angle, target_lf_rebound, MOTOR_ANGLE_DEADBAND) && 
                       CHECK_ANGLE_ARRIVED(motor_rf->measure.total_angle, target_rf_rebound, MOTOR_ANGLE_DEADBAND)) {
                        DART_SET_BIT(3, FLAG_R_REBOUND_REACHED);
                        DART_SET_BIT(3, FLAG_L_REBOUND_REACHED);
                    }
                }
            return;
        }
        break;

            //     // 3508到位判断
            //     if(key==1||key==2){
            //         if(flag_3508_ready == 0 && F_data[0] >= FOCE_3508_STAY1)
            //         {   
            //             flag_3508_ready = 1;
            //         }
            //     }
            //     else if(key==3||key==4){
            //         if(flag_3508_ready == 0 && F_data[0] >= FOCE_3508_STAY2)
            //         {   
            //             flag_3508_ready = 1;
            //         }                    
            //     }
            // // }
            break;
        default:
            break;
    }
    }


#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
