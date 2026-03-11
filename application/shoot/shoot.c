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
#define DM_STEP_VAL 0.00785f // 90度/s 在 200Hz 更新频率下 (90度≈1.57弧度，1.57÷200=0.00785)
#define ROTATE_POWERON_SOFTSTART_MS 700U
#define ROTATE_POWERON_KP_MIN_RATIO 0.0f
#define ROTATE_POWERON_KD_MIN_RATIO 0.0f
float dm_target_angle = ROTATE_1_CHANGE_DARTS_ANGLE;     // 我们希望最终到达的角度
float dm_current_setpoint = ROTATE_1_CHANGE_DARTS_ANGLE; // 当前发送给电机的瞬时角度（插值过程量）
static uint8_t rotate_poweron_softstart_done = 0;
static uint32_t rotate_poweron_softstart_tick = 0;
static float rotate_angle_kp_nominal = 30.0f;
static float rotate_angle_kd_nominal = 0.5f;

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
    // 如果电机未使能（例如在 SHOOT_OFF 模式），持续同步设定值到当前位置
    // 这样当进入 SHOOT_AUTO 时，起点就是当前位置，而不是 0
    if (rotateChageDarts->stop_flag == MOTOR_STOP) {
        dm_current_setpoint = rotateChageDarts->measure.position;
        return; 
    }

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
void servo_magnet_init(int *initFlag)
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
						.Kp = 10, // 10
						.Ki = 0,
						.Kd = 0,
						.MaxOut = 500000,
					},
				.speed_PID =
					{
						.Kp = 5, // 10
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
		.storage_type = USE_STORAGE};
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
						.Kp = 1,
						.Kd = 10.0,
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
	dm_target_angle = ROTATE_2_CHANGE_DARTS_ANGLE; // 设置目标角度为第二发位置
	if(fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f) // 放宽到位判断阈值
	{
		key1_start_tick = 0; // 未到位时重置时间戳
		return;
	}

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
		relay_control(1, 1);
	}
	else {
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(1, 0);
		DART_SET_BIT(1, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(1, FLAG_DART_DROPPED);
	}
}

void setKey2()
{
	dm_target_angle = ROTATE_3_CHANGE_DARTS_ANGLE; // 设置目标角度为第三发位置
	if(fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f)
	{
		key2_start_tick = 0;
		return;
	}

	if (key2_start_tick == 0) {
		key2_start_tick = HAL_GetTick();
		if(key2_start_tick == 0) key2_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key2_start_tick;

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
		relay_control(3, 1);
	}
	else {
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(3, 0);
		DART_SET_BIT(3, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(3, FLAG_DART_DROPPED);
	}
}

void setKey3()
{
	dm_target_angle = ROTATE_4_CHANGE_DARTS_ANGLE; // 设置目标角度为第四发位置
	if(fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f) 
	{
		key3_start_tick = 0;
		return;
	}

	if (key3_start_tick == 0) {
		key3_start_tick = HAL_GetTick();
		if(key3_start_tick == 0) key3_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key3_start_tick;

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
		relay_control(2, 1);
	}
	else {
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(2, 0);
		DART_SET_BIT(2, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(2, FLAG_DART_DROPPED);
	}
}

int initServoMagnet = 0;
int initRotate = 0;

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
	// 从cmd获取控制数据
	SubGetMessage(shoot_sub, &shoot_cmd_recv);
	if(!initServoMagnet){
		servo_magnet_init(&initServoMagnet);
	}
	// if(!initServoMagnet){
	// 	servo_magnet_init(&initServoMagnet);
	// }
	
	if(!initRotate){
		// 阈值放宽至 0.25f，以包容你看到的 0.14 左右的稳态误差
		if (fabs(rotateChageDarts->measure.position) < 0.20f) {
			// 一旦到达靠近 0 的稳态区间，立刻上大刚度锁死
			DMMotorSetKp(rotateChageDarts, rotate_angle_kp_nominal); 
			DMMotorSetKd(rotateChageDarts, rotate_angle_kd_nominal);
			initRotate = 1;
		}
	}

	// BanjiServoStepTest(); // 调用舵机阶梯测试函数
	// ServoStepTest(gripper1_motor);
	rotateSlowMove();
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
			break;
		case BANJI_OFF:
			ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
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
			DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
			//-168058
			// DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);
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
			dm_target_angle = shoot_cmd_recv.rotate_rate;
			// DMMotorSetRef(rotateChageDarts, shoot_cmd_recv.rotate_rate, calculateTff());
			break;
		case ROTATE_AUTO:
			/* code */
			break;
		default:
			break;
		}
		// rotateSlowMove();
		switch (shoot_cmd_recv.GripperTest)
		{
			case 1:
				setKey1();
				break;
			case 2:
				setKey2();
				break;
			case 3:
				setKey3();
				break;
			default:
				break;
		}
		break;
	case SHOOT_AUTO:
		// 这是整个流程的 auto
		{
			DJIMotorEnable(chargeLoader);
			DMMotorEnable(rotateChageDarts);
			rotateSlowMove();
			DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
			uint8_t cur = DartSys.currentStep; 
			if (cur >= 4) return;
			if (cur == 0) {
				DART_SET_BIT(0, FLAG_DART_DROPPED); 
				if (!DART_CHECK_BIT(0, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置，这里设置扳机的位置闭环，setref 为一个值就可以了
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE,MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
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
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(1, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(1, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_2_CHANGE_DARTS_ANGLE, 0.1f)) {
						DART_SET_BIT(1, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(1, MASK_READY_TO_LOAD)) {
					setKey1(); // 关闭电磁铁，放下飞镖
				}
				if(!DART_CHECK_MASK(1, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(1, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
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
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(2, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(2, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_3_CHANGE_DARTS_ANGLE, 0.1f)) {
						DART_SET_BIT(2, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(2, MASK_READY_TO_LOAD)) {
					setKey2();
				}
				if(!DART_CHECK_MASK(2, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(2, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
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
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_LOAD_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(3, FLAG_TRIGGER_AT_LOAD_POS);
					}
				}
				if (!DART_CHECK_BIT(3, FLAG_RELOAD_ROTATED)) {
					// 检测旋转换弹电机到达旋转位置
					if(CHECK_ANGLE_ARRIVED(rotateChageDarts->measure.position, ROTATE_4_CHANGE_DARTS_ANGLE, 0.1f)) {
						DART_SET_BIT(3, FLAG_RELOAD_ROTATED);
					}
				}
				if (DART_CHECK_MASK(3, MASK_READY_TO_LOAD)) {
					setKey3();
				}
				if(!DART_CHECK_MASK(3, MASK_READY_TO_SHOOT)) {
					return;
				}
				if (!DART_CHECK_BIT(3, FLAG_TRIGGER_AT_SHOOT_POS)) {
					// 扳机移动到发射位置
					DJIMotorSetRef(chargeLoader, LOADER_SHOOT_25_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, LOADER_SHOOT_25_ANGLE, MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
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
