#include "shoot.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"
#include "robot_cmd.h"
#include <stdbool.h>


#define DEAD_LINE_LOAD 20
static DJIMotorInstance *chargeLoader; //蓄力丝杆
static DMMotorInstance *rotateChageDarts; //拨盘电机dm
// 扳机舵机
static ServoInstance *banji_motor;
static ServoInstance *gripper1_motor;
static ServoInstance *gripper2_motor;
static ServoInstance *gripper3_motor;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 10;

float loader_origin_angle = 0;
float dead_angle = 2000;
// extern  RC_ctrl_t *rc_data;
float loader_err = 0;

// 2006归位标志位
bool flag_2006_back = false;
// 2006到达打击目标位置标志位
bool flag_2006_target_ready = false;
int read_2006_angle = 1;
float rate = 20000;  //转动速度
int last_goal = 0;  //上次目标
extern goal_of_dart goal;
extern int flag_3508_ready;
extern int flag_arm_sucess;
extern int flag_wait_dart_load_delay;
extern int flag_loadok;
extern int flag_3508_back;
extern int reload;
extern int reload_auto;
extern int flag;
extern int allow;
extern int flag_3508_max;
//初始位置即为16m

// 2006从初始位置开始记圈到16m的数据
#define TOTAL_ANGLE_16M  (0)   //(200859)
// 2006从初始位置开始记圈到25m的数据
#define TOTAL_ANGLE_25M  (852947)//(1053786)

//旋转换弹部分的宏定义,参数全部要重新调
#define dartExistWeight 0
#define dartNoExistWeight 0
#define dartExistLength 0
#define dartNoExistLength 0

// 夹爪和扳机舵机位置宏定义
#define BANJI_ON_ANGLE 0.078
#define BANJI_OFF_ANGLE 0.063
#define GRIPPER_LAY_ANGLE 0
#define GRIPPER_TAKE_ANGLE 0

int key=1;//换弹标志
int step=0;//步骤标志
int f=0;
int first_time=0;

// 计算力矩前馈
static float calculateTff()
{
    float roatateAngle = radian_to_degree_dm(&rotateChageDarts->measure.position);
    float tff_temp = 0;
    float sinTmp0, sinTmp1, sinTmp2, cosTmp;
    arm_sin_cos_f32(roatateAngle, &sinTmp0, &cosTmp);
    arm_sin_cos_f32(roatateAngle + 120.0f, &sinTmp1, &cosTmp);
    arm_sin_cos_f32(roatateAngle - 120.0f, &sinTmp2, &cosTmp);
    tff_temp = dartExistLength * dartExistWeight * (sinTmp0-sinTmp1-sinTmp2);//全满
    tff_temp = dartExistLength * dartExistWeight * (sinTmp0) - dartNoExistWeight * dartNoExistLength * (sinTmp2) - dartExistLength * dartExistWeight * (sinTmp1);//1 空
    tff_temp = dartNoExistLength * dartNoExistWeight * (sinTmp0) - dartNoExistLength * dartNoExistWeight * (sinTmp2) - dartExistLength * dartExistWeight * (sinTmp1);//1,3 空
    tff_temp = dartNoExistLength * dartNoExistWeight * (sinTmp0-sinTmp1-sinTmp2);//全空
    return tff_temp;
}

// 继电器控制函数,新的继电器函数是 PC6，PI6，PI7
void relay_control(unsigned int number,unsigned int state) //继器函数，number编号，state状态，1高0低
{
//PWm丝印第一排从右往左io口
// PE11
// PE13
// PE14
// PC6
    switch (number)
    {
        case 1:
            if(state == 0)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
            else if(state == 1)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
            break;
        case 2:
            if(state == 0)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
            else if(state == 1)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
            break;
        case 3:
            if(state == 0)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
            else if(state == 1)
                HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
            break;
        case 4:
            if(state == 0)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
            else if(state == 1)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

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
    // 1号夹爪
    Servo_Init_Config_s gripper1_motor_config ={
        .pwm_init_config={
            .htim=&htim1,
            .dutyratio=0,
            .channel=TIM_CHANNEL_2,
            .period=0.02,
        },
        .servo_type=PWM_Servo,
    };
    gripper1_motor= ServoInit(&gripper1_motor_config);
    // 2 号夹爪
    Servo_Init_Config_s gripper2_motor_config ={
        .pwm_init_config={
            .htim=&htim1,
            .dutyratio=0,
            .channel=TIM_CHANNEL_3,
            .period=0.02,
        },
        .servo_type=PWM_Servo,
    };
    gripper2_motor= ServoInit(&gripper2_motor_config);
    // 3 号夹爪
    Servo_Init_Config_s gripper3_motor_config ={
        .pwm_init_config={
            .htim=&htim1,
            .dutyratio=0,
            .channel=TIM_CHANNEL_4,
            .period=0.02,
        },
        .servo_type=PWM_Servo,
    };
    gripper3_motor= ServoInit(&gripper3_motor_config);

    Motor_Init_Config_s chargeLoader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 12, // 10
                .Ki = 0,
                .Kd = 1,
                .MaxOut =200,
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
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向 MOTOR_DIRECTION_NORMAL MOTOR_DIRECTION_REVERSE
        },
        .motor_type = M2006, 
        .storage_type = USE_STORAGE
    };       
    chargeLoader = DJIMotorInit(&chargeLoader_config);
    // 达妙电机配置 - MIT 模式
    Motor_Init_Config_s dm_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,             
            .rx_id = 0x00,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 3,
                .Kd = 0.3,
                .Ki = 0.1,
                .Improve = PID_Integral_Limit || PID_ChangingIntegrationRate || PID_Trapezoid_Intergral,
                .IntegralLimit = 1,
            },
            .speed_PID = {
                .Kp = 3,
                .Kd = 0.3,
                .Ki = 0.1,
                .Improve = PID_Integral_Limit || PID_ChangingIntegrationRate || PID_Trapezoid_Intergral,
                .IntegralLimit = 1,
            },
        },
        .motor_type = G6220  // 达妙电机类型
    };
    rotateChageDarts = DMMotorInit(&dm_motor_config, DM_MIT_MODE);
    
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}



void init_angle(){
    DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
    f=1;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // DMMotorSetRef(rotateChageDarts, 3.14,0);
    // 初始化丝杆角度
    if(read_2006_angle==1)
    {
        loader_origin_angle = chargeLoader->measure.total_angle;
        read_2006_angle = 0;
    }
    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    // if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    // {
    //     ServoSetAngle(banji_motor,BANJI_OFF_ANGLE);
    //     DJIMotorStop(chargeLoader);
    //     DMMotorStop(rotateChageDarts);
    // }
    // else // 恢复运行
    // {   
    //     // 扳机的控制
    //     switch(shoot_cmd_recv.banji_mode)
    //     {
    //         case BANJI_OFF:
    //             ServoSetAngle(banji_motor,BANJI_OFF_ANGLE);
    //             break;
    //         case BANJI_ON:
    //             ServoSetAngle(banji_motor,BANJI_ON_ANGLE);
    //             break;
    //         case BANJI_AUTO:

    //             if(flag_arm_sucess == 0 || flag_3508_ready == 0 || !flag_3508_max)
    //             {
    //                 ServoSetAngle(banji_motor,0.070);
    //             }
    //             // 2006到达目标位置，并且3508归位，并且机械臂执行完毕就发射
    //             else if(flag_2006_target_ready == true)
    //             {   
    //                 // 3508归位 并且 机械臂完成放镖 镖体成功装载
    //                 // allow==1时允许发射
    //                 if( flag_3508_back == 1 && flag_arm_sucess == 1 && flag_wait_dart_load_delay == 1 && flag==1)
    //                 {
    //                     ServoSetAngle(banji_motor, 0.078);
    //                     DWT_Delay(2);
    //                     {
    //                         flag_arm_sucess = 0;
    //                         flag_wait_dart_load_delay = 0;
    //                         flag_3508_ready = 0;
    //                         flag_3508_back = 0;
    //                         flag_3508_max=0;
    //                         reload_auto=1;
    //                         // flag_loadok = 0;  
    //                         flag_2006_target_ready = false; 
    //                         flag_2006_back = false;
    //                         if(key==1){
    //                             goal = ANGLE_LOAD;    
    //                         }
    //                     }
    //                 }
    //                 else
    //                     ServoSetAngle(banji_motor,0.063);
    //             }
    //             break;
    //         default:
    //             break;        
    //     }
    // }

    // if(f==0){
    //     init_angle();
    // }
    // switch (shoot_cmd_recv.load_mode)
    // {
    //     case LOAD_STOP:
    //         DJIMotorOuterLoop(chargeLoader, SPEED_LOOP); // 切换到速度环
    //         DJIMotorSetRef(chargeLoader, 0);             // 同时设定参考值为0,这样停止的速度最快
    //         break;
    //     // 自动装载模式
    //     case AUTO_LOAD:
    //         DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);

    //         // 根据传过来的goal参数实现切换    
    //         switch(goal)
    //         {   
    //             // 打击十六米目标
    //             case ANGLE_16M:
    //                 if(flag_arm_sucess == 1 && flag_3508_ready == 1){
    //                     rate = 20000;
    //                     loader_err = chargeLoader->measure.total_angle - loader_origin_angle - TOTAL_ANGLE_16M;
    //                     if (loader_err <= -DEAD_LINE_LOAD)
    //                     {
    //                         DJIMotorSetRef(chargeLoader, rate);   
    //                     }
    //                     else if(loader_err >= DEAD_LINE_LOAD)
    //                     {
    //                         DJIMotorSetRef(chargeLoader, -rate);
    //                     }
    //                     else
    //                     {
    //                         DJIMotorSetRef(chargeLoader, 0);
    //                         if(flag_3508_max){
    //                             flag_2006_target_ready = true;
    //                         }
    //                     }  
    //                 } 
    //                 else{
    //                     DJIMotorSetRef(chargeLoader, 0);
    //                 }             
    //                 break;
    //             // 打击二十米目标，等待测量
    //             case ANGLE_25M:
    //                 if(flag_arm_sucess == 1 && flag_3508_ready == 1){
    //                     rate = 20000;
    //                     loader_err = chargeLoader->measure.total_angle - loader_origin_angle - TOTAL_ANGLE_25M;
    //                     if (loader_err <= -DEAD_LINE_LOAD)
    //                     {
    //                         DJIMotorSetRef(chargeLoader, rate);   
    //                     }
    //                     else if(loader_err >= DEAD_LINE_LOAD)
    //                     {
    //                         DJIMotorSetRef(chargeLoader, -rate);
    //                     }
    //                     else
    //                     {
    //                         DJIMotorSetRef(chargeLoader, 0);
    //                         if(flag_3508_max){
    //                             flag_2006_target_ready = true;
    //                         }
    //                     }                          
    //                 }      
    //                 else{
    //                     DJIMotorSetRef(chargeLoader, 0);
    //                 }     
    //                 break;
    //             // 装载角度模式，此处设置为初始化时的角度
    //             case ANGLE_LOAD:
    //                 rate = 20000;
    //                 if ((chargeLoader->measure.total_angle - loader_origin_angle)<= -DEAD_LINE_LOAD)
    //                 {
    //                     DJIMotorSetRef(chargeLoader, rate);   
    //                 }
    //                 else if((chargeLoader->measure.total_angle - loader_origin_angle)>= DEAD_LINE_LOAD)
    //                 {
    //                     DJIMotorSetRef(chargeLoader, -rate);
    //                 }
    //                 else
    //                 {
    //                     flag_2006_back = true;   
    //                 }             
    //                 break;
    //             default:
    //                 break;
    //         }
    //         break;
    //     case LOADER_TEST:
    //         DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);
    //         DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_rate);
    //         break;
    //     default:
    //         break;
    // }


    switch (shoot_cmd_recv.shoot_mode)
    {
        case SHOOT_OFF:
            ServoSetAngle(banji_motor,BANJI_OFF_ANGLE);
            DJIMotorStop(chargeLoader);
            DMMotorStop(rotateChageDarts);
            break;
        case SHOOT_TEST:
            switch (shoot_cmd_recv.banji_mode)
            {
                case BANJI_OFF:
                    ServoSetAngle(banji_motor,BANJI_OFF_ANGLE);
                    break;
                case BANJI_ON:
                    ServoSetAngle(banji_motor,BANJI_ON_ANGLE);
                    break;
                default:
                    break;
            }
            switch (shoot_cmd_recv.load_mode)
            {
                case LOAD_STOP:
                    DJIMotorOuterLoop(chargeLoader, SPEED_LOOP); // 切换到速度环
                    DJIMotorSetRef(chargeLoader, 0);             // 同时设定
                    break;
                case LOADER_TEST:
                    DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);
                    DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_rate);
                    break;
                case AUTO_LOAD:
                    /* code */
                    break;
                default:
                    break;
            }
            switch (shoot_cmd_recv.rotate_mode)
            {
                case ROTATE_STOP:
                    DMMotorStop(rotateChageDarts);
                    break;
                case ROTATE_TEST:
                    float tff = calculateTff();
                    DMMotorSetRef(rotateChageDarts, shoot_cmd_recv.rotate_rate, tff);
                    break;
                case ROTATE_AUTO:
                    /* code */
                    break;
                default:
                    break;
            }
            DMMotorSetRef(rotateChageDarts, shoot_cmd_recv.rotate_rate, 0);

            break;
        case SHOOT_AUTO:
            //这是整个流程的 auto
            /* code */
            break;
        default:
            break;
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}