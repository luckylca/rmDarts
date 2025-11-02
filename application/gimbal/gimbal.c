#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "jz_motor.h"
#include "user_lib.h"
static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static BMI088Instance *bmi088; // 云台IMU

extern int16_t cyy_test_data;
float cyy_target_angle = 0;
float cyy_init_angle = 0;
int init_flag = 0;
int cyy_target_todo = 1;


void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,//3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50, //100
                .Ki = 1,   //5
                .Kd = 0,    //5
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 200,
                .MaxOut = 800, //500
            },
            .speed_PID = {
                .Kp = 10,  // 50
                .Ki = 1, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP |SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,//MOTOR_DIRECTION_NORMAL 修改
        },
        .motor_type = GM6020};
   
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
   


    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}


void GimbalTask()
{
   
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
     
        break;

    case TWO_YAW:
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw+130); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(yaw_motor, 130);
        break;
    case AUTO_DART:
    //这一部分是将视觉数据转换为电机角度
        DJIMotorEnable(yaw_motor);
        if(cyy_target_todo == 1)
        {
            cyy_target_angle -=0.1 * (cyy_test_data-320)/64.0f;
            cyy_target_angle = float_constrain(cyy_target_angle, -20.0, 20.0);
            cyy_target_todo = 0;
        }
        if((-yaw_motor->measure.total_angle-cyy_target_angle<1)&&(-yaw_motor->measure.total_angle-cyy_target_angle>-1))
        {
            cyy_target_todo=1;
        }
        else
        {
            cyy_target_todo=0;
        }
        // cyy_target_angle += cyy_test_data * 0.000004;
       
        // cyy_target_angle = float_constrain(cyy_target_angle, -170.0, 170.0);
        DJIMotorSetRef(yaw_motor, cyy_target_angle); 
        // DJIMotorSetRef(yaw_motor, 100);
        break;
    default:
        break;
    }


    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}