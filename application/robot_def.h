/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"
#include <math.h> // 需要包含 math.h 以使用 fabsf

#define MOTOR_ANGLE_DEADBAND 100.0f // 允许的误差范围
#define CHECK_ANGLE_ARRIVED(current, target, DEADBAND) (fabsf((current) - (target)) < (DEADBAND))



/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
//#define VISION_USE_UART // 使用串口发送视觉数据

#define VIRSION // 使用视觉数据进行辅助瞄准
// #define REFEREE //接入裁判系统

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

#define BANJI_OPEN_ANGLE 0.098f    // 扳机舵机打开角度
#define BANJI_CLOSE_ANGLE 0.082f // 扳机舵机关闭角度
#define GRIPPER_CLOSE_ANGLE 0.000f // 扳机舵机发射角度
#define GRIPPER_1_LAY_ANGLE 0.111f // 夹爪放置角度
#define GRIPPER_1_NORMAL_ANGLE 0.075f // 夹爪正常角度
#define GRIPPER_2_LAY_ANGLE 0.500f // 夹爪放置角度//600
#define GRIPPER_2_NORMAL_ANGLE 0.0650f // 夹爪正常角度
#define GRIPPER_3_LAY_ANGLE 0.055f // 夹爪放置角度  
#define GRIPPER_3_NORMAL_ANGLE 0.030f // 夹爪正常角度

#define LF_RELOAD_ANGLE  -23500.0f               //左边换弹位置
#define RE_RELOAD_ANGLE  23500.0f             //右边换弹位置
#define LF_1_RELOAD_ANGLE  -25000.0f               //左边换弹位置
#define RE_1_RELOAD_ANGLE  25000.0f             //右边换弹位置
#define LF_2_RELOAD_ANGLE  -24000.0f               //左边换弹位置
#define RE_2_RELOAD_ANGLE  24000.0f             //右边换弹位置
#define LF_3_RELOAD_ANGLE  -24300.0f               //左边换弹位置
#define RE_3_RELOAD_ANGLE  24300.0f             //右边换弹位置
#define LF_CHASSIS_3508_LOAD_ANGLE -39000.0f // 3508蓄力到位角度；39000
#define RF_CHASSIS_3508_LOAD_ANGLE 39000.0f // 3508蓄力到位角度；39000
#define LF_CHASSIS_3508_REBOUND_ANGLE 4400.0f // 3508反弹到位角度 
#define RF_CHASSIS_3508_REBOUND_ANGLE -4400.0f // 3508反弹到位角度

#define RELOAD_TRIGGER_POS 20000.0f
#define RELOAD_TRIGGER_DEADBAND 100.0f

#define ROTATE_1_CHANGE_DARTS_ANGLE 0.0f    // 旋转换弹第一发电机绝对位置,也就是初始值
#define ROTATE_2_CHANGE_DARTS_ANGLE -1.126f // 旋转换弹第二发电机绝对位置
#define ROTATE_3_CHANGE_DARTS_ANGLE -3.180f // 旋转换弹第三发电机绝对位置 0。9046
#define ROTATE_4_CHANGE_DARTS_ANGLE -5.305f // 旋转换弹第四发电机绝对位置3。00

#define ENCODER_LEFT_LIMIT  0x400     // 左边encoder限位值
#define ENCODER_RIGHT_LIMIT 0x4b2    // 右边encoder限位值

// #define YELLOW_25M_SHOOT_ANGLE -385246.7f // 黄色25m发射位置
// #define GREEN_25M_SHOOT_ANGLE -385246.7f //  绿色25m发射位置
// #define BLUE_25M_SHOOT_ANGLE -385246.7f //  蓝色25m发射位置
// #define PURPLE_25M_SHOOT_ANGLE -385246.7f // 紫色25m发射位置

#define YELLOW_25M_SHOOT_ANGLE -380000.0f // 黄色25m发射位置;-115000.7f；-539260.0f
#define GREEN_25M_SHOOT_ANGLE -375000.0f //  绿色25m发射位置;-105000.7f；-.344f
#define BLUE_25M_SHOOT_ANGLE -370000.0f //  蓝色25m发射位置;-95000.7f；110745.789f
#define PURPLE_25M_SHOOT_ANGLE -365000.0f // 紫色25m发射位置;-85000.7f；

#define YELLOW_25M_YAW_ANGLE -41317.6f // 黄色25m发射位置
#define GREEN_25M_YAW_ANGLE 15584.0f //  绿色25m发射位置
#define BLUE_25M_YAW_ANGLE -67444.5f //  蓝色25m发射位置
#define PURPLE_25M_YAW_ANGLE -63924.5f // 紫色25m发射位置

#define R_POSITION_LIMIT -700000.0f // 2006最右位置+++++++++++++++++++限制

#define REFEREE

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_TEST,
    AUTO_MODE,
} chassis_mode_e;    

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_TEST,
    AUTO_DART,

} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_TEST,
    SHOOT_AUTO,
    SHOOT_ON
} shoot_mode_e;
typedef enum
{
    BANJI_OFF = 0, // 摩擦轮关闭
    BANJI_ON,      // 摩擦轮开启
    BANJI_AUTO
} banji_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    AUTO_LOAD,
    LOADER_TEST,
} loader_mode_e;
typedef enum
{
    ROTATE_STOP = 0,  // 停止
    ROTATE_TEST,   // 旋转测试模式
    ROTATE_AUTO,   // 自动旋转模式
} rotate_mode_e;

// 目标宏定义
typedef enum
{   
    // 打击16m目标
    ANGLE_16M = 0,
    // 打击25m目标
    ANGLE_25M,
    // 归位，装载位
    ANGLE_LOAD
} goal_of_dart;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float v_;           // 前进方向速度
    float v1;           // 横移方向速度

    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    double yaw;
    float bottom;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;//总的模式
    loader_mode_e load_mode;//丝杆模式
    banji_mode_e banji_mode;//扳机模式
    rotate_mode_e rotate_mode;//旋转换弹模式
    float shoot_data;
    float rotate_rate;
    float banjiPos;
    int GripperTest;
    float keep_2;
    float err_of_pix;
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)


#endif // !ROBOT_DEF_H