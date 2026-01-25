#include "shoot.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "message_center.h"
#include "remote_control.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "servo_motor.h"
#include <stdbool.h>
#include "status.h"
#include "at24c02.h"
#include "cmsis_os.h"

#define DEAD_LINE_LOAD 20
static DJIMotorInstance *chargeLoader;	  // 蓄力丝杆
static DMMotorInstance *rotateChageDarts; // 拨盘电机dm
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
float rate = 20000; // 转动速度
int last_goal = 0;	// 上次目标
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
// 初始位置即为16m

// 2006从初始位置开始记圈到16m的数据
#define TOTAL_ANGLE_16M (0) //(200859)
// 2006从初始位置开始记圈到25m的数据
#define TOTAL_ANGLE_25M (852947) //(1053786)

// 旋转换弹部分的宏定义,参数全部要重新调
#define dartExistWeight 0
#define dartNoExistWeight 0
#define dartExistLength 0
#define dartNoExistLength 0
#define DM_STEP_VAL 0.2f
float dm_target_angle = ROTATE_1_CHANGE_DARTS_ANGLE;     // 我们希望最终到达的角度
float dm_current_setpoint = ROTATE_1_CHANGE_DARTS_ANGLE; // 当前发送给电机的瞬时角度（插值过程量）
// 计算力矩前馈
// static float calculateTff()
// {
// 	float roatateAngle = radian_to_degree_dm(&rotateChageDarts->measure.position);
// 	float tff_temp = 0;
// 	float sinTmp0, sinTmp1, sinTmp2, cosTmp;
// 	arm_sin_cos_f32(roatateAngle, &sinTmp0, &cosTmp);
// 	arm_sin_cos_f32(roatateAngle + 120.0f, &sinTmp1, &cosTmp);
// 	arm_sin_cos_f32(roatateAngle - 120.0f, &sinTmp2, &cosTmp);
// 	tff_temp =
// 		dartExistLength * dartExistWeight * (sinTmp0 - sinTmp1 - sinTmp2); // 全满
// 	tff_temp = dartExistLength * dartExistWeight * (sinTmp0)-dartNoExistWeight *
// 			   dartNoExistLength * (sinTmp2)-dartExistLength * dartExistWeight *
// 			   (sinTmp1); // 1 空
// 	tff_temp = dartNoExistLength * dartNoExistWeight *
// 			   (sinTmp0)-dartNoExistLength * dartNoExistWeight *
// 			   (sinTmp2)-dartExistLength * dartExistWeight * (sinTmp1); // 1,3 空
// 	tff_temp = dartNoExistLength * dartNoExistWeight *
// 			   (sinTmp0 - sinTmp1 - sinTmp2); // 全空
// 	return tff_temp;
// }
static float calculateTff()
{
    // 1. 获取当前电机角度
    float roatateAngle = radian_to_degree_dm(&rotateChageDarts->measure.position);
    
    // 2. 定义局部变量
    float tff_temp = 0.0f;
    float sin1, sin2, sin3, cosTmp;
    
    // 3. 计算各个夹爪的实际物理角度 (关键修正！)
    // 根据你的描述：电机转到30度时，1号臂到达底部(垂直向下，即0度相位)
    // 所以：Arm1_Angle = Motor_Angle - 30
    float angle1 = roatateAngle - 30.0f;
    float angle2 = angle1 + 120.0f;
    float angle3 = angle1 - 120.0f; // 或者 +240.0f

    // 4. 计算三角函数
    arm_sin_cos_f32(angle1, &sin1, &cosTmp);
    arm_sin_cos_f32(angle2, &sin2, &cosTmp);
    arm_sin_cos_f32(angle3, &sin3, &cosTmp);

    // 5. 确定当前三个臂的质量/力矩系数 (M * g * L)
    // 使用变量分别代表三个臂的 "m*g*L" 系数
    float torque_coeff_1, torque_coeff_2, torque_coeff_3;
    
    float val_full  = dartExistLength * dartExistWeight;   // 满载时的 m*L (注意：这里还没乘g，如果你的Weight是质量kg，最后要乘9.8)
    float val_empty = dartNoExistLength * dartNoExistWeight; // 空载时的 m*L

    // 6. 状态机：根据角度判断谁掉了 (逻辑修正)
    // 注意：这里假设你只正向旋转。如果需要往回转，逻辑一样。
    // 增加了一点点冗余角度防止边界跳变
    
    if (roatateAngle < 30.0f) 
    {
        // --- 阶段 0: 0 ~ 30度 ---
        // 初始状态，全满
        torque_coeff_1 = val_full;
        torque_coeff_2 = val_full;
        torque_coeff_3 = val_full;
    }
    else if (roatateAngle < 150.0f)
    {
        // --- 阶段 1: 30 ~ 150度 ---
        // 1号臂已放下 (空)，2、3号满
        torque_coeff_1 = val_empty;
        torque_coeff_2 = val_full;
        torque_coeff_3 = val_full;
    }
    else if (roatateAngle < 270.0f)
    {
        // --- 阶段 2: 150 ~ 270度 ---
        // 1号空，3号也放下了 (根据顺时针顺序，下一个到底是2还是3，取决于你的机械安装，这里假设间隔120度是3号)
        torque_coeff_1 = val_empty;
        torque_coeff_2 = val_full;
        torque_coeff_3 = val_empty; 
    }
    else
    {
        // --- 阶段 3: > 270度 ---
        // 2号也放下了，全空
        torque_coeff_1 = val_empty;
        torque_coeff_2 = val_empty;
        torque_coeff_3 = val_empty;
    }

    // 7. 计算总负载力矩 (求和！)
    // Torque = (m1*L1*sin1) + (m2*L2*sin2) + (m3*L3*sin3)
    // 假设你的 Weight 变量已经是 力(N) 或者 质量*g
    // 如果 Weight 只是质量(kg)，这里需要乘以 9.8f
    float total_load_torque = (torque_coeff_1 * sin1) + 
                              (torque_coeff_2 * sin2) + 
                              (torque_coeff_3 * sin3);

    // 8. 输出前馈
    // 前馈是要“抵抗”负载，所以通常取反
    tff_temp = -total_load_torque;

    return tff_temp;
}
static void rotateSlowMove(void)
{
    // 1. 线性插值计算 (Ramp)
    float diff = dm_target_angle - dm_current_setpoint;

    // 如果误差大于步长，就走一步
    if (fabs(diff) > DM_STEP_VAL) {
        if (diff > 0) {
            dm_current_setpoint += DM_STEP_VAL;
        } else {
            dm_current_setpoint -= DM_STEP_VAL;
        }
    } else {
        // 误差很小，直接等于目标值
        dm_current_setpoint = dm_target_angle;
    }
    DMMotorSetRef(rotateChageDarts, dm_current_setpoint, calculateTff()); 
}


// 继电器控制函数,新的继电器函数是 PC6，PI6，PI7
void relay_control(
	unsigned int number,
	unsigned int state) // 继器函数，number编号，state状态，1高0低
{
	// PWm丝印第一排从右往左io口
	//  PC6
	// PI6
	// PI7
	switch (number)
	{
	case 1:
		if (state == 0)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		else if (state == 1)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		break;
	case 2:
		if (state == 0)
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_RESET);
		else if (state == 1)
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_6, GPIO_PIN_SET);
		break;
	case 3:
		if (state == 0)
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_RESET);
		else if (state == 1)
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7, GPIO_PIN_SET);
		break;
	default:
		break;
	}
}
int isGripper1Init = 0;
int isGripper2Init = 0;
int isGripper3Init = 0; 
static uint32_t gripper1_wait_tick = 0;
static uint32_t gripper2_wait_tick = 0;
static uint32_t gripper3_wait_tick = 0;
void  servo_magnet_init(int *initFlag)
{
	// 关闭夹爪磁铁
	relay_control(1, 0);
	relay_control(2, 0);
	relay_control(3, 0);

	if (gripper1_wait_tick == 0)
		gripper1_wait_tick = HAL_GetTick();
	
	if (HAL_GetTick() - gripper1_wait_tick < 1500)
		ServoSetAngle(gripper1_motor, GRIPPER_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
		isGripper1Init = 1;
	}

	if (gripper2_wait_tick == 0)
		gripper2_wait_tick = HAL_GetTick();

	if (HAL_GetTick() - gripper2_wait_tick < 1500)
		ServoSetAngle(gripper2_motor, GRIPPER_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		isGripper2Init = 1;
	}

	if (gripper3_wait_tick == 0)
		gripper3_wait_tick = HAL_GetTick();

	if (HAL_GetTick() - gripper3_wait_tick < 1500)
		ServoSetAngle(gripper3_motor, GRIPPER_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		isGripper3Init = 1;
	}
	if(isGripper1Init&&isGripper2Init&&isGripper3Init)
	{
		*initFlag = 1;
	}
}

void ShootInit()
{
	// banji
	Servo_Init_Config_s banji_config = {
		.pwm_init_config =
			{
				.htim = &htim1,
				.dutyratio = 0,
				.channel = TIM_CHANNEL_1,
				.period = 0.02,
			},
		.servo_type = PWM_Servo,
	};
	banji_motor = ServoInit(&banji_config);
	// 1号夹爪
	Servo_Init_Config_s gripper1_motor_config = {
		.pwm_init_config =
			{
				.htim = &htim1,
				.dutyratio = 0,
				.channel = TIM_CHANNEL_2,
				.period = 0.02,
			},
		.servo_type = PWM_Servo,
	};
	gripper1_motor = ServoInit(&gripper1_motor_config);
	// 2 号夹爪
	Servo_Init_Config_s gripper2_motor_config = {
		.pwm_init_config =
			{
				.htim = &htim1,
				.dutyratio = 0,
				.channel = TIM_CHANNEL_3,
				.period = 0.02,
			},
		.servo_type = PWM_Servo,
	};
	gripper2_motor = ServoInit(&gripper2_motor_config);
	// 3 号夹爪
	Servo_Init_Config_s gripper3_motor_config = {
		.pwm_init_config =
			{
				.htim = &htim1,
				.dutyratio = 0,
				.channel = TIM_CHANNEL_4,
				.period = 0.02,
			},
		.servo_type = PWM_Servo,
	};
	gripper3_motor = ServoInit(&gripper3_motor_config);

	Motor_Init_Config_s chargeLoader_config = {
		.can_init_config =
			{
				.can_handle = &hcan1,
				.tx_id = 4,
			},
		.controller_param_init_config =
			{
				.angle_PID =
					{
						.Kp = 15, // 10
						.Ki = 0,
						.Kd = 1,
						.MaxOut = 500000,
					},
				.speed_PID =
					{
						.Kp = 10, // 10
						.Ki = 1,  // 1
						.Kd = 0,
						.Improve = PID_Integral_Limit,
						.IntegralLimit = 5000,
						.MaxOut = 100000,
					},
				.current_PID =
					{
						.Kp = 0.5, // 0.7
						.Ki = 0.1, // 0.1
						.Kd = 0,
						.Improve = PID_Integral_Limit,
						.IntegralLimit = 5000,
						.MaxOut = 5000,
					},
			},
		.controller_setting_init_config =
			{
				.angle_feedback_source = MOTOR_FEED,
				.speed_feedback_source = MOTOR_FEED,
				.outer_loop_type = SPEED_LOOP,
				.close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
				.motor_reverse_flag =
					MOTOR_DIRECTION_REVERSE,
											// MOTOR_DIRECTION_NORMAL
											// MOTOR_DIRECTION_REVERSE
			},
		.motor_type = M2006,
		.storage_type = NO_STORAGE};
	chargeLoader = DJIMotorInit(&chargeLoader_config);
	// 达妙电机配置 - MIT 模式
	Motor_Init_Config_s dm_motor_config = {
		.can_init_config =
			{
				.can_handle = &hcan2,
				.tx_id = 0x01,
				.rx_id = 0x00,
			},
		.controller_setting_init_config =
			{
				.angle_feedback_source = MOTOR_FEED,
				.speed_feedback_source = MOTOR_FEED,
				.outer_loop_type = ANGLE_LOOP,
				.close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
				.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
			},
		.controller_param_init_config =
			{
				.angle_PID =
					{
						.Kp = 30,
						.Kd = 1.0,
						.Ki = 0,
						.Improve = PID_Integral_Limit |
								   PID_ChangingIntegrationRate |
								   PID_Trapezoid_Intergral,
						.IntegralLimit = 1,
					},
				.speed_PID =
					{
						.Kp = 3,
						.Kd = 0.3,
						.Ki = 0.1,
						.Improve = PID_Integral_Limit |
								   PID_ChangingIntegrationRate |
								   PID_Trapezoid_Intergral,
						.IntegralLimit = 1,
					},
			},
		.motor_type = G6220 // 达妙电机类型
	};
	rotateChageDarts = DMMotorInit(&dm_motor_config, DM_MIT_MODE);
	Motor_Recoder_Init_Config_s recoder_init_config = {
		.type = DJI_MOTOR,
		.data.dji = chargeLoader,
	};
	motorRecoderRegister(&recoder_init_config);
	// servo_magnet_init();
	shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
	shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

// void init_angle()
// {
// 	DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
// 	f = 1;
// }


// 将变量提升到函数外部（全局变量），方便在 Ozone Watch 窗口中查看
float servo_test_current_angle = 0.0f; 

/**
 * @brief 通用阶梯式测试舵机函数 (非阻塞)
 * 固定参数：0.0 ~ 0.5, 步长 0.005, 间隔 1000ms
 * 注意：使用静态变量维护时间，同一时间只能测试一个舵机
 * @param servo 舵机实例指针
 */
void ServoStepTest(ServoInstance *servo)
{
    static uint32_t last_move_time = 0;
    const float START_ANGLE = 0.0f;
    const float END_ANGLE = 0.5f;
    const float STEP_SIZE = 0.005f;
    const uint32_t INTERVAL_MS = 1000;

    // 获取当前时间 (ms)
    uint32_t now = HAL_GetTick();

    // 检查是否达到时间间隔
    if (now - last_move_time >= INTERVAL_MS)
    {
        last_move_time = now;

        // 设置舵机角度
        ServoSetAngle(servo, servo_test_current_angle);

        // 增加角度
        servo_test_current_angle += STEP_SIZE;

        // 如果超过最大值，重置为最小值
        if (servo_test_current_angle > END_ANGLE)
        {
            servo_test_current_angle = START_ANGLE;
        }
    }
}

void BanjiServoStepTest()
{
    ServoStepTest(banji_motor);
}

static int isBanjiOpen = 0;
static uint32_t magnet1_wait_tick = 0;
static uint32_t magnet2_wait_tick = 0;
static uint32_t magnet3_wait_tick = 0;

static uint32_t key1_start_tick = 0;
static uint32_t key2_start_tick = 0;
static uint32_t key3_start_tick = 0;

void setKey1()
{
	if (key1_start_tick == 0) {
		key1_start_tick = HAL_GetTick();
		if(key1_start_tick == 0) key1_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key1_start_tick;

	if (dt < 1500) {
		ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
		relay_control(1, 0);
	}
	else if (dt < 2000) { // 1500 + 500
		ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
		relay_control(1, 1);
	}
	else if (dt < 3500) { // 2000 + 1500
		ServoSetAngle(gripper1_motor, GRIPPER_NORMAL_ANGLE);
		relay_control(1, 0);
	}
	else {
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(1, 0);
	}
}

void setKey2()
{
	if (key2_start_tick == 0) {
		key2_start_tick = HAL_GetTick();
		if(key2_start_tick == 0) key2_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key2_start_tick;

	if (dt < 1500) {
		ServoSetAngle(gripper2_motor, GRIPPER_LAY_ANGLE);
		relay_control(2, 0);
	}
	else if (dt < 2000) {
		ServoSetAngle(gripper2_motor, GRIPPER_LAY_ANGLE);
		relay_control(2, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper2_motor, GRIPPER_NORMAL_ANGLE);
		relay_control(2, 0);
	}
	else {
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(2, 0);
	}
}

void setKey3()
{
	if (key3_start_tick == 0) {
		key3_start_tick = HAL_GetTick();
		if(key3_start_tick == 0) key3_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key3_start_tick;

	if (dt < 1500) {
		ServoSetAngle(gripper3_motor, GRIPPER_LAY_ANGLE);
		relay_control(3, 0);
	}
	else if (dt < 2000) {
		ServoSetAngle(gripper3_motor, GRIPPER_LAY_ANGLE);
		relay_control(3, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper3_motor, GRIPPER_NORMAL_ANGLE);
		relay_control(3, 0);
	}
	else {
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(3, 0);
	}
}

int initServoMagnet = 0;


/* 机器人发射机构控制核心任务 */
void ShootTask()
{
	// 从cmd获取控制数据
	SubGetMessage(shoot_sub, &shoot_cmd_recv);
	// DMMotorSetRef(rotateChageDarts, 3.14,0);
	// 初始化丝杆角度
	// if (read_2006_angle == 1)
	// {
	// 	loader_origin_angle = chargeLoader->measure.total_angle;
	// 	read_2006_angle = 0;
	// }
	// 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
	// if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
	// {
	//     ServoSetAngle(banji_motor,BANJI_CLOSE_ANGLE);
	//     DJIMotorStop(chargeLoader);
	//     DMMotorStop(rotateChageDarts);
	// }
	// else // 恢复运行
	// {
	//     // 扳机的控制
	//     switch(shoot_cmd_recv.banji_mode)
	//     {
	//         case BANJI_OFF:
	//             ServoSetAngle(banji_motor,BANJI_CLOSE_ANGLE);
	//             break;
	//         case BANJI_ON:
	//             ServoSetAngle(banji_motor,BANJI_OPEN_ANGLE);
	//             break;
	//         case BANJI_AUTO:
	//             if(flag_arm_sucess == 0 || flag_3508_ready == 0 ||
	//             !flag_3508_max)
	//             {
	//                 ServoSetAngle(banji_motor,0.070);
	//             }
	//             // 2006到达目标位置，并且3508归位，并且机械臂执行完毕就发射
	//             else if(flag_2006_target_ready == true)
	//             {
	//                 // 3508归位 并且 机械臂完成放镖 镖体成功装载
	//                 // allow==1时允许发射
	//                 if( flag_3508_back == 1 && flag_arm_sucess == 1 &&
	//                 flag_wait_dart_load_delay == 1 && flag==1)
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
	//         DJIMotorSetRef(chargeLoader, 0);             //
	//         同时设定参考值为0,这样停止的速度最快 break;
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
	//                     loader_err = chargeLoader->measure.total_angle -
	//                     loader_origin_angle - TOTAL_ANGLE_16M; if (loader_err
	//                     <= -DEAD_LINE_LOAD)
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
	//                     loader_err = chargeLoader->measure.total_angle -
	//                     loader_origin_angle - TOTAL_ANGLE_25M; if (loader_err
	//                     <= -DEAD_LINE_LOAD)
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
	//                 if ((chargeLoader->measure.total_angle -
	//                 loader_origin_angle)<= -DEAD_LINE_LOAD)
	//                 {
	//                     DJIMotorSetRef(chargeLoader, rate);
	//                 }
	//                 else if((chargeLoader->measure.total_angle -
	//                 loader_origin_angle)>= DEAD_LINE_LOAD)
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
	//         DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_data);
	//         break;
	//     default:
	//         break;
	// }
	if(!initServoMagnet){
		servo_magnet_init(&initServoMagnet);
	}
	// BanjiServoStepTest(); // 调用舵机阶梯测试函数
	// ServoStepTest(gripper1_motor);
	switch (shoot_cmd_recv.shoot_mode)
	{
	case SHOOT_OFF:
		ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
		DJIMotorStop(chargeLoader); 
		DMMotorStop(rotateChageDarts);
		break;
	case SHOOT_TEST:
		DJIMotorEnable(chargeLoader);
		DMMotorEnable(rotateChageDarts);
		switch (shoot_cmd_recv.banji_mode)
		{
		case BANJI_ON:
			ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE);

			// if (!isGripper1Open) {
			// 	gripper1_wait_tick = HAL_GetTick();
			// 	isGripper1Open = 1;
			// }
			// if (HAL_GetTick() - gripper1_wait_tick < 1000)
			// 	ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
			// else
			// 	ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);

			// if (!isGripper2Open) {
			// 	gripper2_wait_tick = HAL_GetTick();
			// 	isGripper2Open = 1;
			// }
			// if (HAL_GetTick() - gripper2_wait_tick < 1000)
			// 	ServoSetAngle(gripper2_motor, GRIPPER_LAY_ANGLE);
			// else
			// 	ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);

			// if (!isGripper3Open) {
			// 	gripper3_wait_tick = HAL_GetTick();
			// 	isGripper3Open = 1;
			// }
			// if (HAL_GetTick() - gripper3_wait_tick < 1000)
			// 	ServoSetAngle(gripper3_motor, GRIPPER_LAY_ANGLE);
			// else
			// 	ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
			break;
		case BANJI_OFF:
			ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);

			// if (isGripper1Open) {
			// 	gripper1_wait_tick = HAL_GetTick();
			// 	isGripper1Open = 0;
			// }
			// if (HAL_GetTick() - gripper1_wait_tick < 1000)
			// 	ServoSetAngle(gripper1_motor, GRIPPER_NORMAL_ANGLE);
			// else
			// 	ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);

			// if (isGripper2Open) {
			// 	gripper2_wait_tick = HAL_GetTick();
			// 	isGripper2Open = 0;
			// }
			// if (HAL_GetTick() - gripper2_wait_tick < 1000)
			// 	ServoSetAngle(gripper2_motor, GRIPPER_NORMAL_ANGLE);
			// else
			// 	ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);

			// if (isGripper3Open) {
			// 	gripper3_wait_tick = HAL_GetTick();
			// 	isGripper3Open = 0;
			// }
			// if (HAL_GetTick() - gripper3_wait_tick < 1000)
			// 	ServoSetAngle(gripper3_motor, GRIPPER_NORMAL_ANGLE);
			// else
			// 	ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);

			break;
		case BANJI_AUTO:
			break;
		default:
			break;
		}
		switch (shoot_cmd_recv.load_mode)
		{
		case LOAD_STOP:
			DJIMotorOuterLoop(chargeLoader, SPEED_LOOP); // 切换到速度环
			DJIMotorSetRef(chargeLoader, 0);			 // 同时设定
			break;
		case LOADER_TEST:
			DJIMotorEnable(chargeLoader);
			// DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
			DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);
			DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_data);
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
			DMMotorSetRef(rotateChageDarts, shoot_cmd_recv.rotate_rate, 0);
			break;
		case ROTATE_AUTO:
			/* code */
			break;
		default:
			break;
		}
		switch (shoot_cmd_recv.GripperTest)
		{
			case 0:
				setKey1();
				break;
			case 1:
				setKey2();
				break;
			case 2:
				setKey3();
				break;
			default:
				break;
		}
		break;
	case SHOOT_AUTO:
		// 这是整个流程的 auto
		{
			uint8_t cur = DartSys.currentStep; 
			// 防止数组越界
			if (cur >= 4) return;
			rotateSlowMove(); // 旋转换弹电机慢速运行函数
			if (cur == 0) {
				if (!DART_CHECK_BIT(0, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置，这里设置扳机的位置闭环，setref 为一个值就可以了
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(0, FLAG_TRIGGER_AT_SHOOT_POS);
					}
					return;
				}
				if (DART_CHECK_MASK(0, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
					osDelay(2); // 延时2ms确保扳机打开
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE); // 关闭
					DART_SET_BIT(0, FLAG_FIRED);
				}
				if (DART_CHECK_BIT(0, FLAG_FIRED)) {
					// 第一发镖完成标志位,这个时候开始运行旋转换弹的旋转 30°
					//这里写开始旋转的代码
					dm_target_angle = ROTATE_2_CHANGE_DARTS_ANGLE;
					DartSys.currentStep++;
				}
				return;
			}

			// 第二发镖
			if (cur == 1) {
				if (!DART_CHECK_BIT(1, FLAG_TRIGGER_AT_LOAD_POS)) {
					// 扳机移动到装弹位置
					DJIMotorSetRef(chargeLoader, LOADER_LOAD_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(1, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(1, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_2_CHANGE_DARTS_ANGLE)) {
						DART_SET_BIT(1, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(1, MASK_READY_TO_LOAD)) {
					// 机械臂角度到达位置
					// ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
					// DWT_Delay(2); // 延时2ms确保夹爪放下
					// //这里写关闭电磁铁的代码
					setKey1(); // 关闭电磁铁，放下飞镖
					DART_SET_BIT(1, FLAG_ARM_ANGLE_READY);
					DART_SET_BIT(1, FLAG_DART_DROPPED);
				}
				if(!DART_CHECK_MASK(1, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(1, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(1, FLAG_TRIGGER_AT_SHOOT_POS);
					}
				}
				if (DART_CHECK_MASK(1, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
					osDelay(2); // 延时2ms确保扳机打开
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE); // 关闭
					DART_SET_BIT(1, FLAG_FIRED);
				}
				if (DART_CHECK_BIT(1, FLAG_FIRED)) {
					// 第二发镖完成标志位
					//这里写开始旋转的代码
					dm_target_angle = ROTATE_3_CHANGE_DARTS_ANGLE;
					DartSys.currentStep++;
				}
				return;
			}

			// 第三发镖
			if (cur == 2) {
				if (!DART_CHECK_BIT(2, FLAG_TRIGGER_AT_LOAD_POS)) {
					// 扳机移动到装弹位置
					DJIMotorSetRef(chargeLoader, LOADER_LOAD_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(2, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(2, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_3_CHANGE_DARTS_ANGLE)) {
						DART_SET_BIT(2, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(2, MASK_READY_TO_LOAD)) {
					// 机械臂角度到达位置
					// ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
					// DART_SET_BIT(2, FLAG_ARM_ANGLE_READY);
					// osDelay(500); // 延时2ms确保夹爪放下
					//这里写关闭电磁铁的代码
					setKey2();
					DART_SET_BIT(2, FLAG_ARM_ANGLE_READY);
					DART_SET_BIT(2, FLAG_DART_DROPPED);
				}
				if(!DART_CHECK_MASK(2, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(2, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(2, FLAG_TRIGGER_AT_SHOOT_POS);
					}
				}
				if (DART_CHECK_MASK(2, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
					osDelay(2); // 延时2ms确保扳机打开
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE); // 关闭
					DART_SET_BIT(2, FLAG_FIRED);
				}
				if (DART_CHECK_BIT(2, FLAG_FIRED)) {
					// 第三发镖完成标志位
					//这里写开始旋转的代码
					dm_target_angle = ROTATE_4_CHANGE_DARTS_ANGLE;
					DartSys.currentStep++;
				}
				return;
			}

			// 第四发镖
			if (cur == 3) {
				if (!DART_CHECK_BIT(3, FLAG_TRIGGER_AT_LOAD_POS)) {
					// 扳机移动到装弹位置
					DJIMotorSetRef(chargeLoader, LOADER_LOAD_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(3, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(3, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_4_CHANGE_DARTS_ANGLE)) {
						DART_SET_BIT(3, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(3, MASK_READY_TO_LOAD)) {
					// 机械臂角度到达位置
					// ServoSetAngle(gripper1_motor, GRIPPER_LAY_ANGLE);
					// DART_SET_BIT(3, FLAG_ARM_ANGLE_READY);
					// osDelay(500); // 延时2ms确保夹爪放下
					//这里写关闭电磁铁的代码
					setKey3();
					DART_SET_BIT(3, FLAG_ARM_ANGLE_READY);
					DART_SET_BIT(3, FLAG_DART_DROPPED);
				}
				if(!DART_CHECK_MASK(3, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(3, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE)) {// 到达位置后设置标志位
						DART_SET_BIT(3, FLAG_TRIGGER_AT_SHOOT_POS);
					}
				}
				if (DART_CHECK_MASK(3, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
					osDelay(2); // 延时2ms确保扳机打开
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE); // 关闭
					DART_SET_BIT(3, FLAG_FIRED);
				}
				if (DART_CHECK_BIT(3, FLAG_FIRED)) {
					// 第四发镖完成标志位
					DartSys.currentStep++;
				}
				return;
			}
		}
		break;
	default:
		break;
	}
	PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}