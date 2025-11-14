// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "vision.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "referee_protocol.h"
#include "referee_task.h"
#include "imageRoad.h"
#include "at24c02.h"
#include "servo_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "buzzer.h"
#include <stdbool.h>


// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define armor_f (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define armor_l (1118 * ECD_ANGLE_COEF_DJI)
#define armor_r (5120 * ECD_ANGLE_COEF_DJI)
#define armor_b (3140 * ECD_ANGLE_COEF_DJI)

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD
UART_HandleTypeDef huart2;
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static volatile double *F_data = NULL;
double F_data_1 = 0;
double F_data_2 = 0;
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据


static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static remote_control_t* imageRoad_data;
static ImageRoadRC imageRoad_key[2];

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;

static ServoInstance *arm_motor;

static referee_info_t* referee_info;
int reverse_flag = 0;
int hit_turning = 0;
float flag_vision_ = 0;

static PIDRefs* pid_refs;
// 蜂鸣器buzzer
static bool buzzer_inity_flag = false;
BuzzzerInstance *remot_alarm;
BuzzzerInstance *robotcmd_alarm;
static int alarm_count;
TickType_t startTime;

// uart计数
static int time = 0;
static int last_time=RC_SW_DOWN;
int yaw_control_servo = 0;

// 舵机机械臂完成标志位
int flag_arm_sucess=0;
// 打击目标（默认16m
goal_of_dart goal = ANGLE_LOAD;

extern int flag_3508_ready;
extern bool flag_2006_back;
extern int key;

bool flag_init_goal = false;
int flag=0;//全就位后选择发射
int reload = 0;
int reload_auto=1;
int last_OP=0;
int allow=0;//允许发射

void Change_bottom_position(int num);

struct Vision_angle
{
    float pitch;
    float yaw;
    float flag;
} Vision_angle;

RX_PACKET* rxpack;

PIDInstance Vision_PID = {
    .Kp = 8,
    .Ki = 0.5,
    .Kd = 0,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 1000,
    .MaxOut = 2,
};
float out, last_in;
float T=0.1;
float forward_feed(float in)
{
    out=(in-last_in)/T+in;
    last_in=in;
    return out;
}

void RobotCMDInit()
{

    // 初始化蜂鸣器
    Buzzer_config_s buzzer_config ={
        .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
        .loudness=  0.1, //设置响度
        .octave=  OCTAVE_4, // 设置音阶
    };  

    remot_alarm = BuzzerRegister(&buzzer_config);


    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

    F_data = F_Init(&huart6);

    vision_recv_data = VisionInit(&huart2); // 视觉通信串口，这个不实际占用串口
    
    readAllMotorAngle(); // 从EEPROM加载所有电机总角度数据
    // imageRoad_data = ImageRoadTaskInit(&huart1);

    Vision_angle.pitch = 0;
    Vision_angle.yaw = 0;
    Vision_angle.flag = 0;

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.bottom = 0;
    gimbal_cmd_send.yaw = 0;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{

}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 目前打算是中间统一为测试模式，底部统一为失能，顶部为自动模式
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        chassis_cmd_send.chassis_mode = TEST;//CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode = TEST;
        if (rc_data[TEMP].rc.dial > 0 )// 拨轮打开发射
        {

            shoot_cmd_send.banji_mode = BANJI_ON;
        }
        else
        {
            // 默认锁定
            shoot_cmd_send.banji_mode = BANJI_OFF;
        }
        shoot_cmd_send.shoot_rate = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;    //参数  要改
        chassis_cmd_send.v1 = 30.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1竖直方向
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 
    {
        chassis_cmd_send.chassis_mode =CHASSIS_ZERO_FORCE ;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.banji_mode = BANJI_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = AUTO_MODE;//CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.load_mode = AUTO_LOAD;
        //初始化2006
        if(flag_init_goal==false)
        {
            goal = ANGLE_LOAD;
            flag_init_goal = true;
        }

        //裁判系统接受的数据
        int res=referee_info->DartInfo.dart_info/64%4;
        //手动测试
        // if(key==3||key==2){
        //     goal=ANGLE_16M;
        // }
        // else if(key==1||key==4){
        //     goal=ANGLE_25M;
        // }        
        //2006归位且换弹未完成可更换发射目标
        if(flag_2006_back&&!flag_arm_sucess){
            //裁判系统更改
            if(res==1||res==0){
                goal=ANGLE_16M;     //前哨站
            }
            else if(res==2||res==3){
                goal=ANGLE_16M;     //基地固定or随机
            }
        }
        //发射站状态
        if(referee_info->DartCmd.dart_launch_opening_status==0){
            allow=1;
        }
        else{
            allow=0;
        }

        if(rc_data[TEMP].rc.rocker_r_>200)
        { 
            shoot_cmd_send.shoot_rate =20000;
        }
        else if(rc_data[TEMP].rc.rocker_r_<-200)
        {
            shoot_cmd_send.shoot_rate =-20000;
        }
        else
            shoot_cmd_send.shoot_rate = 0;

        // shoot_cmd_send.shoot_rate += 0.1f * (float)rc_data[TEMP].rc.rocker_r_;    //参数  要改
        chassis_cmd_send.v1 = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1竖直方向
    }


    if(switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        shoot_cmd_send.banji_mode = BANJI_ON_AUTO;
        //自动时控制发射
        if(rc_data[TEMP].rc.dial > 0){
            flag=1;
        }
        else{
            flag=0;
        }
    }
    else if (rc_data[TEMP].rc.dial > 0 )// 拨轮打开发射
    {

        shoot_cmd_send.banji_mode = BANJI_ON;
    }
    else
    {
        // 默认锁定
        shoot_cmd_send.banji_mode = BANJI_OFF;
    }

    // 云台参数,确定云台控制数据
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 
    {   
        gimbal_cmd_send.gimbal_mode = TSET;
        if(gimbal_cmd_send.yaw>30)
        {
            gimbal_cmd_send.yaw=30;
        }
        else if (gimbal_cmd_send.yaw<-50)
        {
            gimbal_cmd_send.yaw=-50;
        }

        if(gimbal_cmd_send.bottom>30)
        {
            gimbal_cmd_send.bottom=30;
        }
        else if (gimbal_cmd_send.bottom<-30)
        {
            gimbal_cmd_send.bottom=-30;
        }
        gimbal_cmd_send.yaw += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
        // gimbal_cmd_send.bottom += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;

        // if(rc_data[TEMP].rc.rocker_l_>400)
        // {
        //     Change_bottom_position(25);
        // }
        // else if(rc_data[TEMP].rc.rocker_l_<-400)
        // {
        //     Change_bottom_position(16);
        // }
        // else
        // {
        //     Change_bottom_position(0);
        // }
        
        // gimbal_cmd_send.yaw = yaw_control_servo;
        


    }

    else if (switch_is_down(rc_data[TEMP].rc.switch_left))// || vision_recv_data->target_state == NO_TARGET
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.banji_mode = BANJI_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");   
    }
    // 云台软件限位
    // 发射参数
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左 侧开关状态[上],
    {
        gimbal_cmd_send.gimbal_mode = TSET;
        gimbal_cmd_send.yaw += 0.001f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.bottom += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;

        if(gimbal_cmd_send.yaw>30)
        {
            gimbal_cmd_send.yaw=30;
        }
        else if (gimbal_cmd_send.yaw<-30)
        {
            gimbal_cmd_send.yaw=-30;
        }

        if(gimbal_cmd_send.bottom>30)
        {
            gimbal_cmd_send.bottom=30;
        }
        else if (gimbal_cmd_send.bottom<-30)
        {
            gimbal_cmd_send.bottom=-30;
        }
        
        
        // gimbal_cmd_send.gimbal_mode = AUTO_DART;
    }                                       

  
}
static void VisionControl()
{
    
}

static void MouseKeySet()
{
    
}


static void ImageRoadSet()
{
   
}                                                   


// 遥控器掉线报警
static void RemoteControl_outline_ALARM()
{
    AlarmSetStatus(remot_alarm, ALARM_ON);
    DWT_Delay(1);
    AlarmSetStatus(remot_alarm, ALARM_OFF);
    DWT_Delay(1);
    

}


static void EmergencyHandler()
{   
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if ((RemoteControlIsOnline()==0)|| robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    { 
        
        alarm_count++;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.banji_mode = BANJI_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    } 
    else 
    {   
        alarm_count = 0;
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

void RobotCMDTask()
{

#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    referee_info = Get_referee_info();
    RemoteControlSet();   
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}

// 拉力传感器任务
void uart6Task()
{
     // RemoteControl_outline_ALARM();
      // // 读取串口六的数据
    uint8_t data[4] = {0};
    if (HAL_UART_Receive(&huart6, data, 4, HAL_MAX_DELAY) == HAL_OK)
    {
            // 处理接收到的数据
    
            HAL_UART_Transmit_IT(&huart6, data, 4);
            
            // 调用你的处理函数
            RemoteControl_outline_ALARM();      
    }
    else
    {
        AlarmSetStatus(remot_alarm, ALARM_OFF);
    }

}