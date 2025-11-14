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

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <stdbool.h>
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

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
extern double F_data[2];
// 3508到位标志位
int flag_3508_ready = 0;
// 完成机械臂动作
extern int flag_arm_sucess ;

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
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.5, // 4.5
                .Ki = 0,  // 0
                .Kd = 0.0001,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
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
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id = 1;//4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;//;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;//
    motor_rf = DJIMotorInit(&chassis_motor_config);
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    // referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI
    referee_data = ReTaskInit(&huart1); // 裁判系统初始化
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
 
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
    }
    else
    {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
    }
    
    // 正常工作,初始化左右电机初始角度
    if(read_original_3508_angle==1)
    {
        original_angle_left = motor_lf->measure.total_angle;
        original_angle_right = motor_rf->measure.total_angle;
        read_original_3508_angle = 0;
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
        case TEST: 
            DJIMotorSetRef(motor_lf, chassis_cmd_recv.v1);//2000 左右能动
            DJIMotorSetRef(motor_rf, chassis_cmd_recv.v1);
            break;
        case AUTO_MODE: 
            // if(flag_2006_back){
                // 当3508还没有到位，并且拉力小于目标拉力时
                if( (flag_3508_ready == 0 || flag_arm_sucess == 0) && flag_2006_target_ready == false)
                {
                    // 等待
                    // if(flag_loadok == 0)
                    // {
                    //     DWT_Delay(2);
                    //     // 飞镖装载完毕标志位
                    //     flag_loadok = 1;
                    // } 
                    if(key==1||key==2)
                        if(F_data[0] <= FOCE_3508_STAY1)
                        {
                            DJIMotorSetRef(motor_lf, SPEED_3508);
                            DJIMotorSetRef(motor_rf, SPEED_3508);
                        }
                        else
                        {
                            DJIMotorSetRef(motor_lf, 0.5*SPEED_3508);
                            DJIMotorSetRef(motor_rf, 0.5*SPEED_3508);                   
                        }
                    else if(key==3||key==4){
                        if(F_data[0] <= FOCE_3508_STAY2)
                        {
                            DJIMotorSetRef(motor_lf, SPEED_3508);
                            DJIMotorSetRef(motor_rf, SPEED_3508);
                        }
                        else
                        {
                            DJIMotorSetRef(motor_lf, 0.5*SPEED_3508);
                            DJIMotorSetRef(motor_rf, 0.5*SPEED_3508);                   
                        }
                    }
                }

                else if(flag_arm_sucess == 1 && flag_3508_ready == 1)
                {   
                    // 放镖延时
                    if(flag_wait_dart_load_delay == 0)
                    {
                        // goal=ANGLE_16M;
                        DWT_Delay(1);
                        flag_wait_dart_load_delay=1;
                    }

                    if(!flag_3508_max){
                        if(F_data[0] <= FOCE_3508_MAX)
                        {
                            DJIMotorSetRef(motor_lf, SPEED_3508);
                            DJIMotorSetRef(motor_rf, SPEED_3508);
                        }
                        else
                        {
                            DJIMotorSetRef(motor_lf, 0.5*SPEED_3508);
                            DJIMotorSetRef(motor_rf, 0.5*SPEED_3508); 
                            flag_3508_max=1;                  
                        }     
                    }

                    // 如果2006到位，电机回到原位准备发射
                    if(flag_2006_target_ready == true && flag_3508_back == 0 && flag_3508_max)
                    {
                        // 计算误差
                        err_of_original_angle = motor_lf->measure.total_angle - original_angle_left;
                        // 归位时不受力所以直接3508到位后拉力给0
                        if(err_of_original_angle > 0)
                        {
                            DJIMotorSetRef(motor_lf, -SPEED_3508);
                            DJIMotorSetRef(motor_rf, -SPEED_3508);
                        }
                        else
                        {
                            DJIMotorSetRef(motor_lf, 0);
                            DJIMotorSetRef(motor_rf, 0);
                            // 3508归位标志位
                            flag_3508_back = 1;
                        }   
                    }
                    else if(flag_2006_target_ready == false){
                        if(F_data[0] <= FOCE_3508_MAX)
                        {
                            DJIMotorSetRef(motor_lf, SPEED_3508);
                            DJIMotorSetRef(motor_rf, SPEED_3508);
                        }
                        else
                        {
                            DJIMotorSetRef(motor_lf, 0.5*SPEED_3508);
                            DJIMotorSetRef(motor_rf, 0.5*SPEED_3508);                   
                        }                        
                    }
                }
                else
                {
                    DJIMotorSetRef(motor_lf, 0);
                    DJIMotorSetRef(motor_rf, 0);
                }

                // 3508到位判断
                if(key==1||key==2){
                    if(flag_3508_ready == 0 && F_data[0] >= FOCE_3508_STAY1)
                    {   
                        flag_3508_ready = 1;
                    }
                }
                else if(key==3||key==4){
                    if(flag_3508_ready == 0 && F_data[0] >= FOCE_3508_STAY2)
                    {   
                        flag_3508_ready = 1;
                    }                    
                }
            // }
            break;
        default:
            break;
    }



#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
