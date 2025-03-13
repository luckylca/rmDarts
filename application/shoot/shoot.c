#include "shoot.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"
/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *loader; // 拨盘电机
static ServoInstance *banji_motor;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 10;

float loader_origin_angle = 0;
float dead_angle = 2000;
extern RC_ctrl_t *rc_data;

int flag_2006 = 0;
float rate = 20000;  //转动速度
int last_goal = 0;  //上次目标
extern int goal;
extern int flag_3508;
extern int flag_servo;
extern int flag_delay;
extern int flag_set;
extern int flag_pause;
extern int flag_back;


void ShootInit()
{
    // banji
    Servo_Init_Config_s banji_config ={
        .pwm_init_config={
            .htim=&htim1,
            .dutyratio=0,
            .channel=TIM_CHANNEL_1,
            .period=0.02,
        },
        .servo_type=PWM_Servo,
    };
    banji_motor= ServoInit(&banji_config);


    // 2006
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 12, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 15, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向 MOTOR_DIRECTION_NORMAL MOTOR_DIRECTION_REVERSE
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        ServoSetAngle(banji_motor,0.065);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {   
         // 扳机的控制
        switch(shoot_cmd_recv.banji_mode)
        {
        case BANJI_OFF:
            ServoSetAngle(banji_motor,0.063);
            break;
        case BANJI_ON:
            if(flag_servo == 0){
                ServoSetAngle(banji_motor,0.070);
            }
            else if(flag_2006 == 1){
                ServoSetAngle(banji_motor,0.076);
            }
            
            // ServoSetAngle(banji_motor,0.074);
            if(flag_back == 1){
                flag_servo = 0;
                flag_delay = 0;
                flag_3508 = 0;
                flag_set = 0;
                flag_pause = 0;  
                flag_2006 = 0;              
            }

            break;
        default:
            break;        
        }
        DJIMotorEnable(loader);
    }

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    case LOAD_NORMAL:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        if(flag_servo == 1){
            if(last_goal == 0 && goal == 1)
            {
                rate = 20000;
                loader_origin_angle = loader->measure.total_angle;
                if ((loader->measure.total_angle - loader_origin_angle) <= dead_angle)
                {
                    DJIMotorSetRef(loader, rate);
                    
                }
                else{
                    DJIMotorSetRef(loader, 0);
                    last_goal = goal;
                    flag_2006 = 1;
                }                
                
            }
            else if(last_goal == 1 && goal == 0)
            {
                rate = -20000;
                loader_origin_angle = loader->measure.total_angle;
                if((loader_origin_angle - loader->measure.total_angle ) <= dead_angle){
                    DJIMotorSetRef(loader, rate);
                }
                else{
                    DJIMotorSetRef(loader, 0);
                    last_goal = goal;
                    flag_2006 = 1;
                }   
            }
            else{
                flag_2006 = 1;
            }

        }

        // DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate);
        break;
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate);
        break;
    default:
        break;
       
    }

    // 设置舵机的开关
    // if (shoot_cmd_recv.banji_mode == BANJI_ON)
    // {
    //     ServoSetAngle(banji_motor,0.075);
    // }
    // else 
    // {
    //      ServoSetAngle(banji_motor,0.065);       
    // }



    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}