// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "vision.h"
#include "status.h"
// module
#include "remote_control.h"
#include "mc6c.h"
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
#include "encoder.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "buzzer.h"
#include "user_lib.h"
#include <stdbool.h>



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
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息

static MC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static volatile double *F_data_1 = NULL;
static volatile double *F_data_2 = NULL;
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

extern EncoderInstance *encoder;

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

void RobotCMDInit()
{

    // 初始化蜂鸣器
    Buzzer_config_s buzzer_config ={
        .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
        .loudness=  0.1, //设置响度
        .octave=  OCTAVE_4, // 设置音阶
    };  

    remot_alarm = BuzzerRegister(&buzzer_config);


    // rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    rc_data = MCControlInit(&huart3);

    F_data_1 = F_Init(&huart1);
    // F_data_2 = F_Init(&huart6);
    vision_recv_data = VisionInit(&huart2); // 视觉通信串口，这个不实际占用串口
    
    // readAllMotorAngle(); // 从EEPROM加载所有电机总角度数据
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
int i = 0;
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    #ifdef MC_SBUS
    // 目前打算是中间统一为测试模式，底部统一为失能，顶部为自动模式
    if (mc_data_change(rc_data[TEMP].switch_r)==RC_SW_MID) 
    {
        int16_t rocker_r1 = rc_data[TEMP].rocker_r1;
        chassis_cmd_send.chassis_mode = CHASSIS_TEST;//CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode = SHOOT_TEST;
        shoot_cmd_send.rotate_mode = ROTATE_TEST;
        shoot_cmd_send.load_mode = LOADER_TEST;
        gimbal_cmd_send.gimbal_mode = GIMBAL_TEST;
        if(fabs(rc_data[TEMP].rocker_l1)<50)
        {
            rc_data[TEMP].rocker_l1=0;
        }
        if(fabs(rocker_r1)<50)
        {
            rocker_r1=0;
        }
        if(fabs(rc_data[TEMP].rocker_r_)<50)
        {
            rc_data[TEMP].rocker_r_=0;
        }
        if(fabs(rc_data[TEMP].rocker_l_)<50)
        {
            rc_data[TEMP].rocker_l_=0;
        }
        if(rocker_r1==-960)
        {
            rocker_r1=0;
        }
        // shoot_cmd_send.shoot_data = -100.0f * (float)rc_data[TEMP].rocker_l1; 
        shoot_cmd_send.shoot_data -= 0.8f * (float)rc_data[TEMP].rocker_l1; 
        // chassis_cmd_send.v1 -= 0.1f * (float)rc_data[TEMP].rocker_r1; // 1竖直方向
        chassis_cmd_send.v1 = -15.0f * (float)rocker_r1; // 1竖直方向，速度 12000
        gimbal_cmd_send.yaw += 0.5f * (float)rc_data[TEMP].rocker_l_;//底盘的位置
        // shoot_cmd_send.rotate_rate += 30.0f * (float)rc_data[TEMP].rocker_r_; // 右水平,换弹旋转的速度，参数依旧要改
        shoot_cmd_send.rotate_rate += 0.000005f*(float)rc_data[TEMP].rocker_r_; // 右水平,换弹旋转的速度，参数依旧要改

        if(rc_data[TEMP].none[0] == 0xc8 && rc_data[TEMP].none[1] != 0xc8 && rc_data[TEMP].none[1] != 0x708)
        {
            shoot_cmd_send.shoot_data = YELLOW_25M_SHOOT_ANGLE;
            // gimbal_cmd_send.yaw = YELLOW_25M_YAW_ANGLE;
        }
        else if(rc_data[TEMP].none[0] == 0x708 && rc_data[TEMP].none[1] != 0xc8 && rc_data[TEMP].none[1] != 0x708)
        {
            shoot_cmd_send.GripperTest=1;
            // gimbal_cmd_send.yaw = GREEN_25M_YAW_ANGLE;
        }
        else if(rc_data[TEMP].none[1] == 0xc8 && rc_data[TEMP].none[0] != 0xc8 && rc_data[TEMP].none[0] != 0x708)
            // shoot_cmd_send.shoot_data = BLUE_25M_SHOOT_ANGLE;
        {
            shoot_cmd_send.GripperTest=2;
            // gimbal_cmd_send.yaw = BLUE_25M_YAW_ANGLE;
        }
        else if(rc_data[TEMP].none[1] == 0x708 && rc_data[TEMP].none[0] != 0xc8 && rc_data[TEMP].none[0] != 0x708)
            // shoot_cmd_send.shoot_data = PURPLE_25M_SHOOT_ANGLE;
        {
            shoot_cmd_send.GripperTest=3;
            // gimbal_cmd_send.yaw = PURPLE_25M_YAW_ANGLE;
        }
        
        #ifdef VIRSION
        gimbal_cmd_send.yaw -= 1.5f * vision_recv_data->err_of_pix;
        #endif // DEBUG
    }
    else if (mc_data_change(rc_data[TEMP].switch_r)==RC_SW_DOWN) // 
    {
        chassis_cmd_send.chassis_mode =CHASSIS_ZERO_FORCE ;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.rotate_mode = ROTATE_STOP;
    }
    else if (mc_data_change(rc_data[TEMP].switch_r)==RC_SW_UP)
    {
        shoot_cmd_send.shoot_mode = SHOOT_AUTO; 
        gimbal_cmd_send.gimbal_mode = AUTO_DART;
        chassis_cmd_send.chassis_mode = AUTO_MODE;
    }

    //下面是对每个模式的细化设置，就是在 TEST 模式下的对某个模块做其他测试
    if (mc_data_change(rc_data[TEMP].switch_l)==RC_SW_MID) // 
    {   
        shoot_cmd_send.banji_mode = BANJI_ON;
    }
    else if (mc_data_change(rc_data[TEMP].switch_l)==RC_SW_DOWN)// || vision_recv_data->target_state == NO_TARGET
    {
        shoot_cmd_send.banji_mode = BANJI_OFF;
    }
    else if (mc_data_change(rc_data[TEMP].switch_l)==RC_SW_UP) // 左 侧开关状态[上],
    {
        shoot_cmd_send.banji_mode = BANJI_OFF;
    }                        
    #endif

    #ifdef DBUS
    // 目前打算是中间统一为测试模式，底部统一为失能，顶部为自动模式
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_TEST;//CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode = SHOOT_TEST;
        shoot_cmd_send.rotate_mode = ROTATE_TEST;
        shoot_cmd_send.load_mode = LOADER_TEST;
        gimbal_cmd_send.gimbal_mode = GIMBAL_TEST;
        if (rc_data[TEMP].rc.dial > 0 )// 拨轮打开发射
        {
            shoot_cmd_send.banji_mode = BANJI_ON;
        }
        else
        {
            shoot_cmd_send.banji_mode = BANJI_OFF;
        }
        shoot_cmd_send.shoot_data = 20.0f * (float)rc_data[TEMP].rc.rocker_l1;    //参数要改
        chassis_cmd_send.v1 -= 0.1f * (float)rc_data[TEMP].rc.rocker_r1; // 1竖直方向
        gimbal_cmd_send.yaw = 15.0f * (float)rc_data[TEMP].rc.rocker_l_;//底盘的位置
        shoot_cmd_send.rotate_rate += 30.0f * (float)rc_data[TEMP].rc.rocker_r_; // 右水平,换弹旋转的速度，参数依旧要改
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 
    {
        chassis_cmd_send.chassis_mode =CHASSIS_ZERO_FORCE ;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.banji_mode = BANJI_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.rotate_mode = ROTATE_STOP;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = AUTO_MODE;//CHASSIS_FOLLOW_GIMBAL_YAW;
        shoot_cmd_send.shoot_mode = SHOOT_AUTO;
        shoot_cmd_send.load_mode = AUTO_LOAD;
        shoot_cmd_send.banji_mode = BANJI_AUTO;
        shoot_cmd_send.rotate_mode = ROTATE_AUTO;
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
            shoot_cmd_send.shoot_data =20000;
        }
        else if(rc_data[TEMP].rc.rocker_r_<-200)
        {
            shoot_cmd_send.shoot_data =-20000;
        }
        else
            shoot_cmd_send.shoot_data = 0;

        // shoot_cmd_send.shoot_data += 0.1f * (float)rc_data[TEMP].rc.rocker_r_;    //参数  要改
        // chassis_cmd_send.v1 = 20.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1竖直方向
    }

    //下面是对每个模式的细化设置，就是在 TEST 模式下的对某个模块做其他测试
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 
    {   
        shoot_cmd_send.banji_mode = BANJI_OFF;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left))// || vision_recv_data->target_state == NO_TARGET
    {
        shoot_cmd_send.banji_mode = BANJI_ON;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左 侧开关状态[上],
    {
        //自动模式
    }                                       
    #endif
    // yaw轴限幅 - 根据encoder绝对值限幅
    // {
    //     static float last_valid_yaw = 0.0f; // 上一次有效的 yaw 值
    //     static uint8_t yaw_limit_init = 0;   // 初始化标志


    //     // 第一次运行时，初始化 last_valid_yaw 为当前值
    //     if (!yaw_limit_init) {
    //         last_valid_yaw = gimbal_cmd_send.yaw;
    //         yaw_limit_init = 1;
    //     }

    //     uint16_t enc_pos = encoder->measure.position;
    //     float new_yaw = gimbal_cmd_send.yaw;

    //     // 超过右限位encoder：只能往左减，不能继续往右加
    //     if (enc_pos > ENCODER_RIGHT_LIMIT) {
    //         if (new_yaw > last_valid_yaw) {
    //             // 继续往右越界，不更新
    //             gimbal_cmd_send.yaw = last_valid_yaw;
    //         } else {
    //             // 往回走，允许更新
    //             last_valid_yaw = new_yaw;
    //         }
    //     }
    //     // 超过左限位encoder：只能往右加，不能继续往左减
    //     else if (enc_pos < ENCODER_LEFT_LIMIT) {
    //         if (new_yaw < last_valid_yaw) {
    //             // 继续往左越界，不更新
    //             gimbal_cmd_send.yaw = last_valid_yaw;
    //         } else {
    //             // 往回走，允许更新
    //             last_valid_yaw = new_yaw;
    //         }
    //     } else {
    //         // 在范围内，正常更新
    //         last_valid_yaw = new_yaw;
    //     }
    // }

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
    uint8_t remote_online = 1;
    uint16_t switch_right = RC_SW_DOWN;

#ifdef MC_SBUS
    remote_online = MCControlIsOnline();
    switch_right = mc_data_change(rc_data[TEMP].switch_r);
#endif

#ifdef DBUS
    remote_online = RemoteControlIsOnline();
    switch_right = rc_data[TEMP].rc.switch_right;
#endif

    if ((!remote_online) ||
        (switch_right != RC_SW_UP && switch_right != RC_SW_MID && switch_right != RC_SW_DOWN))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.v_ = 0.0f;
        chassis_cmd_send.v1 = 0.0f;

        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        gimbal_cmd_send.bottom = 0.0f;

        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        shoot_cmd_send.banji_mode = BANJI_OFF;
        shoot_cmd_send.rotate_mode = ROTATE_STOP;
        shoot_cmd_send.shoot_data = 0.0f;
        shoot_cmd_send.rotate_rate = 0.0f;
        shoot_cmd_send.banjiPos = 0.0f;
        shoot_cmd_send.GripperTest = 0;

        RemoteControl_outline_ALARM();
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
    VisionSend();
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
    // 1. 准备接收缓冲区，文档规定回传为10个字节 
    uint8_t rxBuffer[10] = {0}; 
    uint8_t sendData[5] = {17,66,62,17,13};
    
    // 2. 发送数据 (建议先用阻塞发送，确保发完再切接收)
    // 如果有 RS485 控制引脚：
    // RS485_TX_ENABLE(); 
     // 超时设为10ms足够
    
    // 3. 切换到接收模式
    // RS485_RX_ENABLE();
    
    // 4. 接收数据：读取10个字节，设置超时为100ms，不要用 MAX_DELAY
    if (HAL_UART_Receive(&huart1, rxBuffer, 10, 100) == HAL_OK)
    {
        // 5. 校验数据头 (0x11, 0x42) 和 结束符 (0x0D) 
        if(rxBuffer[0] == 0x11 && rxBuffer[1] == 0x42 && rxBuffer[9] == 0x0D)
        {
            // 6. 解析重量数据 X1~X5 
            // 公式：(X5-0x30)*65536 + ... + (X1-0x30)
            int32_t weight = 0;
            weight += (rxBuffer[6] - 0x30) * 65536; // X5 (高位)
            weight += (rxBuffer[5] - 0x30) * 4096;  // X4
            weight += (rxBuffer[4] - 0x30) * 256;   // X3
            weight += (rxBuffer[3] - 0x30) * 16;    // X2
            weight += (rxBuffer[2] - 0x30);         // X1 (低位)
            
            // 7. 处理符号位 (X6 的 bit2) 
            // X6 是 rxBuffer[7]
            if ((rxBuffer[7] & 0x04) != 0) // 检查 Bit 2
            {
                weight = -weight; // 如果 Bit 2 是 1，则是负数
            }

            // 数据获取成功，执行你的逻辑
            // printf("Current Weight: %d\n", weight);
            RemoteControl_outline_ALARM(); 
        }
        else 
        {
            // 数据格式不对（校验错误）
             AlarmSetStatus(remot_alarm, ALARM_OFF);
        }
    }
    else
    {
        // 接收超时（断连）
        AlarmSetStatus(remot_alarm, ALARM_OFF);
    }
}