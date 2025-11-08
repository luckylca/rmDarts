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
/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *loader; // 拨盘电机
static DJIMotorInstance *loader_1; //同步上下
static DJIMotorInstance *loader_2; //左右
static DJIMotorInstance *loader_3; //限位
static DMMotorInstance *rotateChageDarts; //拨盘电机dm
// 扳机舵机
static ServoInstance *banji_motor;

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

//换弹位置
#define  TOTAL_ANGLE_C1  0   //中间左右
#define  TOTAL_ANGLE_C2  150000-1   //中间上下
#define TOTAL_ANGLE_L  770000-1    //左
#define TOTAL_ANGLE_W  100000-1    //上
#define TOTAL_ANGLE_R  -770000+1    //右
#define TOTAL_ANGLE_D1  400000-1    //下1
#define TOTAL_ANGLE_D2  680000-1    //下2
#define TOTAL_ANGLE_Loc 430000-1    //发射位置

int key=1;//换弹标志
int step=0;//步骤标志
int f=0;
int first_time=0;


void relay_control(unsigned int number,unsigned int state) //继电器函数，number编号，state状态，1高0低
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
    Motor_Init_Config_s loader_config_3 = {
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
            // .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006 // 英雄使用m3508
    };       
    loader_3 = DJIMotorInit(&loader_config_3);
    // 达妙电机配置 - MIT 模式
    Motor_Init_Config_s dm_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,  // 确认这是正确的 CAN 总线
            .tx_id = 0x01,             // 确认这是达妙电机的正确 ID
            .rx_id = 0x00,
        },
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 300,
                .Ki = 0,
                .Kd = 0.05,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = G6220  // 达妙电机类型
    };
    rotateChageDarts = DMMotorInit(&dm_motor_config, DM_MIT_MODE);
    
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}



void init_angle(){
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorOuterLoop(loader_1, ANGLE_LOOP);
    DJIMotorOuterLoop(loader_2, ANGLE_LOOP);
    DJIMotorSetRef(loader, 0);
    DJIMotorSetRef(loader_1, 0);
    DJIMotorSetRef(loader_2, 0);
    f=1;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    DMMotorSetRef(rotateChageDarts, 3.14,0);
    // 初始化丝杆角度
    if(read_2006_angle==1)
    {
        loader_origin_angle = loader_3->measure.total_angle;
        read_2006_angle = 0;
    }
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
                ServoSetAngle(banji_motor,0.078);
                break;
            case BANJI_ON_AUTO:

                if(flag_arm_sucess == 0 || flag_3508_ready == 0 || !flag_3508_max)
                {
                    ServoSetAngle(banji_motor,0.070);
                }
                // 2006到达目标位置，并且3508归位，并且机械臂执行完毕就发射
                else if(flag_2006_target_ready == true)
                {   
                    // 3508归位 并且 机械臂完成放镖 镖体成功装载
                    // allow==1时允许发射
                    if( flag_3508_back == 1 && flag_arm_sucess == 1 && flag_wait_dart_load_delay == 1 && flag==1)
                    {
                        ServoSetAngle(banji_motor, 0.078);
                        DWT_Delay(2);
                        {
                            flag_arm_sucess = 0;
                            flag_wait_dart_load_delay = 0;
                            flag_3508_ready = 0;
                            flag_3508_back = 0;
                            flag_3508_max=0;
                            reload_auto=1;
                            // flag_loadok = 0;  
                            flag_2006_target_ready = false; 
                            flag_2006_back = false;
                            if(key==1){
                                goal = ANGLE_LOAD;    
                            }
                                      
                        }

                    }
                    else
                        ServoSetAngle(banji_motor,0.063);
                }
                break;
            default:
                break;        
        }
        DJIMotorEnable(loader);
    }

    if(f==0){
        init_angle();
    }
    switch (shoot_cmd_recv.load_mode)
    {
        // 停止拨盘
        case LOAD_STOP:
            DJIMotorOuterLoop(loader_3, SPEED_LOOP); // 切换到速度环
            DJIMotorSetRef(loader_3, 0);             // 同时设定参考值为0,这样停止的速度最快
            break;
        // 自动装载模式
        case AUTO_LOAD:
            DJIMotorOuterLoop(loader_3, SPEED_LOOP);
    
            if(f&&reload_auto&&!flag_arm_sucess){
                DJIMotorOuterLoop(loader, ANGLE_LOOP);
                DJIMotorOuterLoop(loader_1, ANGLE_LOOP);
                DJIMotorOuterLoop(loader_2, ANGLE_LOOP);

                switch (key)
                {
                case 1://第一发不用换弹
                    if(flag_3508_ready){
                        key=4;
                        reload_auto=0;
                        flag_arm_sucess=1;                        
                    }
                    break;
                case 2: //第二发左上
                    if(step==0){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                            step++;
                        }
                    }
                    else if(step==1){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_L);
                        if(loader_2->measure.total_angle>=TOTAL_ANGLE_L){
                        step++;  
                        }
                    }    
                    else if(step==2){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_D1);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_D1);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_D1&&loader_1->measure.total_angle>=TOTAL_ANGLE_D1){
                            step++;
                            relay_control(2,1);
                            relay_control(1,0);
                        }
                        
                    }
                    else if(step==3){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                        if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                            step++;
                        }
                    }
                    else if(step==4){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                        if(loader_2->measure.total_angle<=TOTAL_ANGLE_C1){
                        step++;  
                        }
                    }
                    else if(step==5){
                        if(flag_3508_ready){                        
                            DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                            DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                            if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                                relay_control(2,0);
                                step++;
                            }
                        }
                    }
                    else if(step==6){         
                        f=0;
                        reload_auto=0;
                        step=0;
                        key++;
                        flag_arm_sucess=1;
                        
                    }
                    break;
                case 3:
                    if(step==0){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                            step++;
                        }
                    }
                    else if(step==1){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_L);
                        if(loader_2->measure.total_angle>=TOTAL_ANGLE_L){
                            step++;  
                            relay_control(3,1);
                        }
                    }    
                    else if(step==2){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_D2);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_D2);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_D2&&loader_1->measure.total_angle>=TOTAL_ANGLE_D2){
                            step++;
                            
                        }
                        
                    }
                    else if(step==3){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                        if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                            step++;
                        }
                    }
                    else if(step==4){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                        if(loader_2->measure.total_angle<=TOTAL_ANGLE_C1){
                        step++;  
                        }
                    }
                    else if(step==5){
                        if(flag_3508_ready){                        
                            DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                            DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                            if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                                relay_control(3,0);
                                step++;
                            }
                        }
                    }
                    else if(step==6){                     
                        f=0;
                        reload_auto=0;
                        step=0;
                        key++;
                        flag_arm_sucess=1;
                    }
                    break;
                case 4:
                    if(step==0){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                            step++;
                        }
                    }
                    else if(step==1){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_R);
                        if(loader_2->measure.total_angle<=TOTAL_ANGLE_R){
                            step++;  
                            relay_control(3,1);                           
                        }
                    }    
                    else if(step==2){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_D2);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_D2);
                        if(loader->measure.total_angle>=TOTAL_ANGLE_D2&&loader_1->measure.total_angle>=TOTAL_ANGLE_D2){
                            step++;
                        }
                        
                    }
                    else if(step==3){
                        DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                        DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                        if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                            step++;
                        }
                    }
                    else if(step==4){
                        DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                        if(loader_2->measure.total_angle>=TOTAL_ANGLE_C1){
                            step++;  
                        }
                    }
                    else if(step==5){
                        if(flag_3508_ready){                        
                            DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                            DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                            if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                                relay_control(3,0);
                                step++;
                            }
                        }
                    }
                    else if(step==6){             
                        f=0;
                        reload_auto=0;
                        step=0;
                        key=1;
                        flag_arm_sucess=1;
                    }
                    break;
                default:
                    break;
                }
            }
            // 根据传过来的goal参数实现切换    
            switch(goal)
            {   
                // 打击十六米目标
                case ANGLE_16M:
                    if(flag_arm_sucess == 1 && flag_3508_ready == 1){
                        rate = 20000;
                        loader_err = loader_3->measure.total_angle - loader_origin_angle - TOTAL_ANGLE_16M;
                        if (loader_err <= -DEAD_LINE_LOAD)
                        {
                            DJIMotorSetRef(loader_3, rate);   
                        }
                        else if(loader_err >= DEAD_LINE_LOAD)
                        {
                            DJIMotorSetRef(loader_3, -rate);
                        }
                        else
                        {
                            DJIMotorSetRef(loader_3, 0);
                            if(flag_3508_max){
                                flag_2006_target_ready = true;
                            }
                        }  
                    } 
                    else{
                        DJIMotorSetRef(loader_3, 0);
                    }             
                    break;
                // 打击二十米目标，等待测量
                case ANGLE_25M:
                    if(flag_arm_sucess == 1 && flag_3508_ready == 1){
                        rate = 20000;
                        loader_err = loader_3->measure.total_angle - loader_origin_angle - TOTAL_ANGLE_25M;
                        if (loader_err <= -DEAD_LINE_LOAD)
                        {
                            DJIMotorSetRef(loader_3, rate);   
                        }
                        else if(loader_err >= DEAD_LINE_LOAD)
                        {
                            DJIMotorSetRef(loader_3, -rate);
                        }
                        else
                        {
                            DJIMotorSetRef(loader_3, 0);
                            if(flag_3508_max){
                                flag_2006_target_ready = true;
                            }
                        }                          
                    }      
                    else{
                        DJIMotorSetRef(loader_3, 0);
                    }     
                    break;
                // 装载角度模式，此处设置为初始化时的角度
                case ANGLE_LOAD:
                    rate = 20000;
                    if ((loader_3->measure.total_angle - loader_origin_angle)<= -DEAD_LINE_LOAD)
                    {
                        DJIMotorSetRef(loader_3, rate);   
                    }
                    else if((loader_3->measure.total_angle - loader_origin_angle)>= DEAD_LINE_LOAD)
                    {
                        DJIMotorSetRef(loader_3, -rate);
                    }
                    else
                    {
                        DJIMotorSetRef(loader, 0);
                        flag_2006_back = true;   
                        
                    }             
                    break;
                default:
                    break;
            }
        case TEST:
            DJIMotorOuterLoop(loader_3, SPEED_LOOP);
            DJIMotorSetRef(loader_3, shoot_cmd_recv.shoot_rate);
            break;


    case LOAD_NORMAL:
        DJIMotorOuterLoop(loader_3, SPEED_LOOP);
        DJIMotorSetRef(loader_3, shoot_cmd_recv.shoot_rate);
        relay_control(2,1);
        if(f&&reload){
            DJIMotorOuterLoop(loader, ANGLE_LOOP);
            DJIMotorOuterLoop(loader_1, ANGLE_LOOP);
            DJIMotorOuterLoop(loader_2, ANGLE_LOOP);
            switch (key)
            {
            case 1://第一发不用换弹
                key++;
                reload=0;
                // flag_arm_sucess=1;
                break;
            case 2: //第二发左上
                if(step==0){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                        step++;
                    }
                }
                else if(step==1){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_L);
                    if(loader_2->measure.total_angle>=TOTAL_ANGLE_L){
                    step++;  
                    }
                }    
                else if(step==2){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_D1);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_D1);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_D1&&loader_1->measure.total_angle>=TOTAL_ANGLE_D1){
                        step++;
                        relay_control(2,1);
                    }
                    
                }
                else if(step==3){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                    if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                        step++;
                    }
                }
                else if(step==4){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                    if(loader_2->measure.total_angle<=TOTAL_ANGLE_C1){
                    step++;  
                    }
                }
                else if(step==5){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                        step++;
                        DWT_Delay(1);
                        // flag_arm_sucess=1;
                        relay_control(2,0);
                    }
                }
                else if(step==6){
                    //自动的话需等待 flag_wait_dart_load_delay或者3508归位
                    f=0;
                    reload=0;
                    step=0;
                    key++;
                }
                break;
            case 3:
                if(step==0){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                        step++;
                    }
                }
                else if(step==1){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_L);
                    if(loader_2->measure.total_angle>=TOTAL_ANGLE_L){
                    step++;  
                    }
                }    
                else if(step==2){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_D2);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_D2);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_D2&&loader_1->measure.total_angle>=TOTAL_ANGLE_D2){
                        step++;
                        relay_control(3,1);
                    }
                    
                }
                else if(step==3){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                    if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                        step++;
                    }
                }
                else if(step==4){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                    if(loader_2->measure.total_angle<=TOTAL_ANGLE_C1){
                    step++;  
                    }
                }
                else if(step==5){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                        step++;
                        DWT_Delay(1);
                        // flag_arm_sucess=1;
                        relay_control(3,0);
                    }
                }
                else if(step==6){
                    //自动的话需等待 flag_wait_dart_load_delay或者3508归位
                    f=0;
                    reload=0;
                    step=0;
                    key++;
                }
                break;
            case 4:
                if(step==0){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_C2);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_C2);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_C2&&loader_1->measure.total_angle>=TOTAL_ANGLE_C2){
                        step++;
                    }
                }
                else if(step==1){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_R);
                    if(loader_2->measure.total_angle<=TOTAL_ANGLE_R){
                        step++;  
                    }
                }    
                else if(step==2){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_D2);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_D2);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_D2&&loader_1->measure.total_angle>=TOTAL_ANGLE_D2){
                        step++;
                        relay_control(3,1);
                    }
                    
                }
                else if(step==3){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_W);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_W);
                    if(loader->measure.total_angle<=TOTAL_ANGLE_W&&loader_1->measure.total_angle<=TOTAL_ANGLE_W){
                        step++;
                    }
                }
                else if(step==4){
                    DJIMotorSetRef(loader_2, TOTAL_ANGLE_C1);
                    if(loader_2->measure.total_angle>=TOTAL_ANGLE_C1){
                    step++;  
                    }
                }
                else if(step==5){
                    DJIMotorSetRef(loader, TOTAL_ANGLE_Loc);
                    DJIMotorSetRef(loader_1, TOTAL_ANGLE_Loc);
                    if(loader->measure.total_angle>=TOTAL_ANGLE_Loc&&loader_1->measure.total_angle>=TOTAL_ANGLE_Loc){
                        step++;
                        DWT_Delay(1);
                        // flag_arm_sucess=1;
                        relay_control(3,0);
                    }
                }
                else if(step==6){
                    //自动的话需等待 flag_wait_dart_load_delay或者3508归位
                    f=0;
                    reload=0;
                    step=0;
                    key=1;
                }
                break;
            default:
                break;
            }
        }
        break;
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate);
        break;
    default:
        break;
       
    }


    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}