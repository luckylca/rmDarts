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
#include <math.h>
#include <stdbool.h>
#include "status.h"
#include "at24c02.h"
#include "cmsis_os.h"
#include "bsp_log.h"
#include "referee_protocol.h"
#include "rm_referee.h"

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

extern referee_info_t* referee_info;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 10;

// 调试用全局变量
float debug_motor_angle = 0;
float debug_calc_tff = 0;
float debug_motor_torque = 0;
uint8_t debug_num_darts = 3;

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
#define DM_STEP_VAL 0.0157f          // 最大速度: 180度/秒
#define DM_ACCEL_VAL 0.000157f       // 加速度: 每周期增加0.000157 rad
#define DM_STOP_EPSILON 0.0025f      // 到位后直接锁定目标，避免末端抖动
#define ROTATE_POWERON_SOFTSTART_MS 700U
#define ROTATE_POWERON_KP_MIN_RATIO 0.0f
#define ROTATE_POWERON_KD_MIN_RATIO 0.0f
float dm_target_angle = ROTATE_1_CHANGE_DARTS_ANGLE;     // 我们希望最终到达的角度
float dm_current_setpoint = ROTATE_1_CHANGE_DARTS_ANGLE; // 当前发送给电机的瞬时角度（插值过程量）
static uint8_t rotate_poweron_softstart_done = 0;
static uint32_t rotate_poweron_softstart_tick = 0;
static float rotate_angle_kp_nominal = 30.0f;
static float rotate_angle_kd_nominal = 1.0f;

typedef enum {
	RELOAD_IDLE = 0,
	RELOAD_WAIT_TRIGGER_POS,
	RELOAD_RUN_KEY,
	RELOAD_WAIT_TRIGGER_BACK,
} ReloadState_e;

static ReloadState_e reload_state = RELOAD_IDLE;
static uint8_t reload_key = 0;
static int last_gripper_cmd = 0;
static uint8_t hold_rotate_after_reload = 0;
static uint8_t hold_loader_after_reload = 0;



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
    #define K_GRAVITY 0.1343f
    #define ZERO_POINT_DEG 118.4f

    float motor_angle_rad = rotateChageDarts->measure.position;
    float motor_angle_deg = motor_angle_rad * 57.2957795f;

    while (motor_angle_deg > 180.0f) motor_angle_deg -= 360.0f;
    while (motor_angle_deg < -180.0f) motor_angle_deg += 360.0f;

    float arm1_angle_deg = motor_angle_deg - ZERO_POINT_DEG;

    uint8_t num_objects = debug_num_darts;

    debug_motor_angle = motor_angle_deg;
    debug_motor_torque = rotateChageDarts->measure.torque;

    if (num_objects == 0 || num_objects == 3) {
        debug_calc_tff = 0.0f;
        return 0.0f;
    }

    float sin_val, cos_val;
    arm_sin_cos_f32(arm1_angle_deg, &sin_val, &cos_val);

    float tff;
    if (num_objects == 2) {
        tff = K_GRAVITY * sin_val;
    } else {
        float arm3_angle_deg = arm1_angle_deg - 240.0f;
        arm_sin_cos_f32(arm3_angle_deg, &sin_val, &cos_val);
        tff = -K_GRAVITY * sin_val;
    }

    debug_calc_tff = tff;
    return tff;
}
static float rotate_current_speed = 0; // 当前速度

static void rotateSlowMove(void)
{
	// 如果电机未使能（例如在 SHOOT_OFF 模式），持续同步设定值到当前位置
	// 这样当进入 SHOOT_AUTO 时，起点就是当前位置，而不是 0
	if (rotateChageDarts->stop_flag == MOTOR_STOP) {
		dm_current_setpoint = rotateChageDarts->measure.position;
		rotate_current_speed = 0;
		return;
	}

	float diff = dm_target_angle - dm_current_setpoint;
	float abs_diff = fabsf(diff);

	if (abs_diff <= DM_STOP_EPSILON) {
		dm_current_setpoint = dm_target_angle;
		rotate_current_speed = 0;
		DMMotorSetRef(rotateChageDarts, dm_current_setpoint, calculateTff());
		return;
	}

	float target_speed = sqrtf(2.0f * DM_ACCEL_VAL * abs_diff);
	if (target_speed > DM_STEP_VAL) {
		target_speed = DM_STEP_VAL;
	}

	if (rotate_current_speed < target_speed) {
		rotate_current_speed += DM_ACCEL_VAL;
		if (rotate_current_speed > target_speed) {
			rotate_current_speed = target_speed;
		}
	} else {
		rotate_current_speed -= DM_ACCEL_VAL;
		if (rotate_current_speed < target_speed) {
			rotate_current_speed = target_speed;
		}
	}

	if (diff > 0.0f) {
		dm_current_setpoint += rotate_current_speed;
		if (dm_current_setpoint > dm_target_angle) {
			dm_current_setpoint = dm_target_angle;
		}
	} else {
		dm_current_setpoint -= rotate_current_speed;
		if (dm_current_setpoint < dm_target_angle) {
			dm_current_setpoint = dm_target_angle;
		}
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
		ServoSetAngle(gripper2_motor, GRIPPER_1_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		isGripper1Init = 1;
	}

	if (gripper2_wait_tick == 0)
		gripper2_wait_tick = HAL_GetTick();

	if (HAL_GetTick() - gripper2_wait_tick < 1500)
		ServoSetAngle(gripper3_motor, GRIPPER_2_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		isGripper2Init = 1;
	}

	if (gripper3_wait_tick == 0)
		gripper3_wait_tick = HAL_GetTick();

	if (HAL_GetTick() - gripper3_wait_tick < 1500)
		ServoSetAngle(gripper1_motor, GRIPPER_3_NORMAL_ANGLE);
	else
	{
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
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
						.MaxOut = 300000,
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
						.MaxOut = 7000,
					},
			},
		.controller_setting_init_config =
			{
				.angle_feedback_source = MOTOR_FEED,
				.speed_feedback_source = MOTOR_FEED,
				.outer_loop_type = SPEED_LOOP,
				.close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
				.motor_reverse_flag =
					MOTOR_DIRECTION_NORMAL,
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
						.Kp = 2,
						.Kd = 10.0,
						.Ki = 0.1,
						.Improve = PID_Integral_Limit |
								   PID_ChangingIntegrationRate |
								   PID_Trapezoid_Intergral,
						.IntegralLimit = 1,
					},
				.speed_PID =
					{
						.Kp = 3,
						.Kd = 0.3,
						.Ki = 1.0,
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
	shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
	shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

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
    const uint32_t INTERVAL_MS = 1500;

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

static int isBanjiOpen = 0;
static uint32_t magnet1_wait_tick = 0;
static uint32_t magnet2_wait_tick = 0;
static uint32_t magnet3_wait_tick = 0;

static uint32_t key1_start_tick = 0;
static uint32_t key2_start_tick = 0;
static uint32_t key3_start_tick = 0;

static void setKey1(void);
static void setKey2(void);
static void setKey3(void);

static void clearKeyDoneFlag(uint8_t key)
{
	if (key == 1)
	{
		DART_CLEAR_BIT(1, FLAG_ARM_ANGLE_READY | FLAG_DART_DROPPED);
	}
	else if (key == 2)
	{
		DART_CLEAR_BIT(2, FLAG_ARM_ANGLE_READY | FLAG_DART_DROPPED);
	}
	else if (key == 3)
	{
		DART_CLEAR_BIT(3, FLAG_ARM_ANGLE_READY | FLAG_DART_DROPPED);
	}
}

static uint8_t isKeyDone(uint8_t key)
{
	if (key == 1)
	{
		return DART_CHECK_BIT(1, FLAG_ARM_ANGLE_READY) && DART_CHECK_BIT(1, FLAG_DART_DROPPED);
	}
	if (key == 2)
	{
		return DART_CHECK_BIT(2, FLAG_ARM_ANGLE_READY) && DART_CHECK_BIT(2, FLAG_DART_DROPPED);
	}
	if (key == 3)
	{
		return DART_CHECK_BIT(3, FLAG_ARM_ANGLE_READY) && DART_CHECK_BIT(3, FLAG_DART_DROPPED);
	}
	return 0;
}

static void runKey(uint8_t key)
{
	if (key == 1)
	{
		setKey1();
	}
	else if (key == 2)
	{
		setKey2();
	}
	else if (key == 3)
	{
		setKey3();
	}
}

static void resetKeyTicks(void)
{
	key1_start_tick = 0;
	key2_start_tick = 0;
	key3_start_tick = 0;
}

void setKey1()
{
	dm_target_angle = ROTATE_2_CHANGE_DARTS_ANGLE; // 设置目标角度为第二发位置
	if(fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.15f) // 放宽到位判断阈值
	{
		key1_start_tick = 0; // 未到位时重置时间戳
		return;
	}

	if (key1_start_tick == 0) {
		key1_start_tick = HAL_GetTick();
		if(key1_start_tick == 0) key1_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key1_start_tick;

	if (dt < 1000) {
		ServoSetAngle(gripper2_motor, GRIPPER_1_LAY_ANGLE);
		relay_control(3, 0);
	}
	else if (dt < 1500) { // 1500 + 500
		ServoSetAngle(gripper2_motor, GRIPPER_1_LAY_ANGLE);
		relay_control(3, 1);
	}
	else if (dt < 3500) { // 2000 + 1500
		ServoSetAngle(gripper2_motor, GRIPPER_1_NORMAL_ANGLE);
		relay_control(3, 1);
	}
	else {
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(3, 0);
		DART_SET_BIT(1, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(1, FLAG_DART_DROPPED);
		DART_SET_BIT(1, FLAG_RELOAD_ROTATED);//先默认置位
	}
}
void setKey1Auto()
{
	dm_target_angle = ROTATE_2_CHANGE_DARTS_ANGLE; // 设置目标角度为第二发位置
	if ((fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f) || (!DART_CHECK_MASK(1, MASK_READY_TO_LOAD)))
	{
		key1_start_tick = 0; // 未到位时重置时间戳
		return;
	}

	if (key1_start_tick == 0) {
		key1_start_tick = HAL_GetTick();
		if(key1_start_tick == 0) key1_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key1_start_tick;

	if (dt < 1000) {
		ServoSetAngle(gripper2_motor, GRIPPER_1_LAY_ANGLE);
		relay_control(3, 0);
	}
	else if (dt < 1500) { // 1500 + 500
		ServoSetAngle(gripper2_motor, GRIPPER_1_LAY_ANGLE);
		relay_control(3, 1);
	}
	else if (dt < 3500) { // 2000 + 1500
		ServoSetAngle(gripper2_motor, GRIPPER_1_NORMAL_ANGLE);
		relay_control(3, 1);
	}
	else {
		ServoSetAngle(gripper2_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(3, 0);
		DART_SET_BIT(1, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(1, FLAG_DART_DROPPED);
		DART_SET_BIT(1, FLAG_RELOAD_ROTATED);//先默认置位
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
		ServoSetAngle(gripper3_motor, GRIPPER_2_LAY_ANGLE);
		relay_control(1, 0);
	}
	else if (dt < 2000) {
		ServoSetAngle(gripper3_motor, GRIPPER_2_NORMAL_ANGLE);
		relay_control(1, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper3_motor, GRIPPER_2_NORMAL_ANGLE);
		relay_control(1, 1);
	}
	else {
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(1, 0);
		DART_SET_BIT(2, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(2, FLAG_DART_DROPPED);
	}
}
void setKey2Auto()
{
	dm_target_angle = ROTATE_3_CHANGE_DARTS_ANGLE; // 设置目标角度为第三发位置
	if ((fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f) || (!DART_CHECK_MASK(2, MASK_READY_TO_LOAD)))
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
		ServoSetAngle(gripper3_motor, GRIPPER_2_LAY_ANGLE);
		relay_control(1, 0);
	}
	else if (dt < 2000) {
		ServoSetAngle(gripper3_motor, GRIPPER_2_NORMAL_ANGLE);
		relay_control(1, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper3_motor, GRIPPER_2_NORMAL_ANGLE);
		relay_control(1, 1);
	}
	else {
		ServoSetAngle(gripper3_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(1, 0);
		DART_SET_BIT(2, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(2, FLAG_DART_DROPPED);
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

	if (dt < 1000) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_LAY_ANGLE);
		relay_control(2, 0);
	}
	else if (dt < 1500) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_LAY_ANGLE);
		relay_control(2, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_NORMAL_ANGLE);
		relay_control(2, 1);
	}
	else {
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(2, 0);
		DART_SET_BIT(3, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(3, FLAG_DART_DROPPED);
	}
}
void setKey3Auto()
{
	dm_target_angle = ROTATE_4_CHANGE_DARTS_ANGLE; // 设置目标角度为第四发位置
	if ((fabs(rotateChageDarts->measure.position - dm_target_angle) > 0.1f) || (!DART_CHECK_MASK(3, MASK_READY_TO_LOAD)))
	{
		key3_start_tick = 0;
		return;
	}

	if (key3_start_tick == 0) {
		key3_start_tick = HAL_GetTick();
		if(key3_start_tick == 0) key3_start_tick = 1;
	}

	uint32_t dt = HAL_GetTick() - key3_start_tick;

	if (dt < 1000) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_LAY_ANGLE);
		relay_control(2, 0);
	}
	else if (dt < 1500) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_LAY_ANGLE);
		relay_control(2, 1);
	}
	else if (dt < 3500) {
		ServoSetAngle(gripper1_motor, GRIPPER_3_NORMAL_ANGLE);
		relay_control(2, 1);
	}
	else {
		ServoSetAngle(gripper1_motor, GRIPPER_CLOSE_ANGLE);
		relay_control(2, 0);
		DART_SET_BIT(3, FLAG_ARM_ANGLE_READY);
		DART_SET_BIT(3, FLAG_DART_DROPPED);
	}
}
int initServoMagnet = 0;
int initRotate = 0;

float getCurrentAngel(int key){
	if(key == 1){
		return ROTATE_2_CHANGE_DARTS_ANGLE;
	}
	else if(key == 2){
		return ROTATE_3_CHANGE_DARTS_ANGLE;
	}
	else if(key == 3){
		return ROTATE_4_CHANGE_DARTS_ANGLE;
	}
	return 0.0f;
}

float getBackPosAngle(int key){
	if(key == 1){
		return GREEN_25M_SHOOT_ANGLE;
	}
	else if(key == 2){
		return BLUE_25M_SHOOT_ANGLE;
	}
	else if(key == 3){
		return PURPLE_25M_SHOOT_ANGLE;
	}
	return YELLOW_25M_SHOOT_ANGLE;
}

int cmd;
int tmpa=0;
int start_0_time = 0;
int start_1_time = 0;
int start_2_time = 0;
int start_3_time = 0;
int tmpb = 0;
extern int i;
/* 机器人发射机构控制核心任务 */
void ShootTask()
{
	// 从cmd获取控制数据
	SubGetMessage(shoot_sub, &shoot_cmd_recv);
	if(!initServoMagnet){
		servo_magnet_init(&initServoMagnet);
	}
	if(!initRotate){
		static uint32_t rotate_in_range_tick = 0;
		if (fabs(rotateChageDarts->measure.position) < 0.15f) {
			if (rotate_in_range_tick == 0) {
				rotate_in_range_tick = HAL_GetTick(); // 记录进入阈值的时间戳
			} else if (HAL_GetTick() - rotate_in_range_tick > 1000) { // 持续 1 秒
				DMMotorSetKp(rotateChageDarts, rotate_angle_kp_nominal); 
				DMMotorSetKd(rotateChageDarts, rotate_angle_kd_nominal);
				initRotate = 1;
			}
		} else {
			rotate_in_range_tick = 0;
		}
	}
	// ServoSetAngle(gripper3_motor,1.0f);
	// BanjiServoStepTest(); // 调用舵机阶梯测试函数
	// ServoStepTest(gripper2_motor);
	// relay_control(1, 1);
	// relay_control(2, 1);
	// relay_control(3, 0);

	rotateSlowMove();
	switch (shoot_cmd_recv.shoot_mode)
	{
	case SHOOT_OFF:
		ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE);
		DJIMotorStop(chargeLoader); 
		DMMotorStop(rotateChageDarts);
		hold_rotate_after_reload = 0;
		hold_loader_after_reload = 0;
		break;
	case SHOOT_TEST:
		DJIMotorEnable(chargeLoader);
		DMMotorEnable(rotateChageDarts);
		/*
		{
			cmd = shoot_cmd_recv.GripperTest;
			if (cmd != last_gripper_cmd)
			{
				if (cmd >= 1 && cmd <= 3 && reload_state == RELOAD_IDLE)
				{
					reload_key = (uint8_t)cmd;
					resetKeyTicks();
					clearKeyDoneFlag(reload_key);
					reload_state = RELOAD_WAIT_TRIGGER_POS;
					hold_rotate_after_reload = 0;
					hold_loader_after_reload = 0;
				}
				last_gripper_cmd = cmd;
			}

			if (reload_state != RELOAD_IDLE)
			{
				DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
				switch (reload_state)
				{
				case RELOAD_WAIT_TRIGGER_POS:
					dm_target_angle = getCurrentAngel(cmd);
					DJIMotorSetRef(chargeLoader, RELOAD_TRIGGER_POS);
					if (CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, RELOAD_TRIGGER_POS, RELOAD_TRIGGER_DEADBAND))
					{
						reload_state = RELOAD_RUN_KEY;
					}
					break;
				case RELOAD_RUN_KEY:
					dm_target_angle = getCurrentAngel(cmd);
					runKey(reload_key);
					if (isKeyDone(reload_key))
					{
						reload_state = RELOAD_WAIT_TRIGGER_BACK;
					}
					break;
				case RELOAD_WAIT_TRIGGER_BACK:
					dm_target_angle = getCurrentAngel(cmd);				
					DJIMotorSetRef(chargeLoader, getBackPosAngle(cmd));
					if (CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, getBackPosAngle(cmd), RELOAD_TRIGGER_DEADBAND))
					{
						reload_state = RELOAD_IDLE;
						reload_key = 0;
						hold_rotate_after_reload = 1;
						hold_loader_after_reload = 1;
					}
					break;
				default:
					reload_state = RELOAD_IDLE;
					reload_key = 0;
					break;
				}
				break;
			}
		}
		*/
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
			// if (hold_loader_after_reload)
			// {
			// 	DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
			// 	DJIMotorSetRef(chargeLoader, getBackPosAngle(cmd));
			// }
			// else
			// {
			// 	DJIMotorOuterLoop(chargeLoader, SPEED_LOOP); // 切换到速度环
			// 	DJIMotorSetRef(chargeLoader, 0);			 // 同时设定
			// }
			DJIMotorStop(chargeLoader);
			break;
		case LOADER_TEST:
			DJIMotorEnable(chargeLoader);
			DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
			// DJIMotorOuterLoop(chargeLoader, SPEED_LOOP);
			DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_data);
			// if(hold_loader_after_reload)
			// 	DJIMotorSetRef(chargeLoader, getBackPosAngle(cmd));
			// else
			// 	DJIMotorSetRef(chargeLoader, shoot_cmd_recv.shoot_data);
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
			// if (hold_rotate_after_reload)
			// {
			// 	dm_target_angle = getCurrentAngel(cmd);
			// }
			// else
			// {
			// 	dm_target_angle = shoot_cmd_recv.rotate_rate;
			// }
			// //力矩正方向是面对镖架的顺时针
			dm_target_angle = shoot_cmd_recv.rotate_rate;
			// DMMotorSetRef(rotateChageDarts, shoot_cmd_recv.rotate_rate, calculateTff());
			break;
		case ROTATE_AUTO:
			/* code */
			break;
		default:
			break;
		}
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
		DJIMotorEnable(chargeLoader);
		DMMotorEnable(rotateChageDarts);
		DJIMotorOuterLoop(chargeLoader, ANGLE_LOOP);
		int cur = DartSys.currentStep;
		#ifdef REFEREE
			//1是关闭，2 是正在开启，0 是已经开启
			if(referee_info->DartCmd.dart_launch_opening_status == 1)
				break;
		#endif
		if(tmpa==1) {
			break;
		}
		switch (cur) {
			case 0:
			{
				DART_SET_BIT(0, FLAG_DART_DROPPED); 
				if (!DART_CHECK_BIT(0, FLAG_TRIGGER_AT_SHOOT_POS)) {
					DJIMotorSetRef(chargeLoader, YELLOW_25M_SHOOT_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, YELLOW_25M_SHOOT_ANGLE,MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(0, FLAG_TRIGGER_AT_SHOOT_POS);
					}
					break;
				}
				if (DART_CHECK_MASK(0, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					tmpb=1;
					if(start_0_time == 0) {
						start_0_time = HAL_GetTick();
					}
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE);
					uint32_t dt = HAL_GetTick() - start_0_time;
					if(dt < 1000) {
						break; // 延时200ms确保扳机打开
					}
					tmpb=2;
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE); // 关闭
					DART_SET_BIT(0, FLAG_FIRED);
					i=0;
					DartSys.currentStep++;
				}
				break;
			}
			case 1:
			{
				setKey1Auto();
				if(!DART_CHECK_BIT(1,FLAG_DART_DROPPED))
					break;
				if (!DART_CHECK_BIT(1, FLAG_TRIGGER_AT_SHOOT_POS)) {
					DJIMotorSetRef(chargeLoader, GREEN_25M_SHOOT_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, GREEN_25M_SHOOT_ANGLE,MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(1, FLAG_TRIGGER_AT_SHOOT_POS);
					}
					break;
				}
				if (DART_CHECK_MASK(1, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					if(start_1_time == 0) {
						start_1_time = HAL_GetTick();
					}
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE);
					uint32_t dt = HAL_GetTick() - start_1_time;
					if(dt < 1000) {
						break; // 延时200ms确保扳机打开
					}
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE); // 关闭
					DART_SET_BIT(1, FLAG_FIRED);
					i=0;
					DartSys.currentStep++;
				}
				break;
			}
			case 2:
			{
				setKey2Auto();
				if(!DART_CHECK_BIT(2,FLAG_DART_DROPPED))
					break;
				if (!DART_CHECK_BIT(2, FLAG_TRIGGER_AT_SHOOT_POS)) {
					DJIMotorSetRef(chargeLoader, BLUE_25M_SHOOT_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, BLUE_25M_SHOOT_ANGLE,MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(2, FLAG_TRIGGER_AT_SHOOT_POS);
					}
					break;
				}
				if (DART_CHECK_MASK(2, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					if(start_2_time == 0) {
						start_2_time = HAL_GetTick();
					}
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE);
					uint32_t dt = HAL_GetTick() - start_2_time;
					if(dt < 1000) {
						break; // 延时200ms确保扳机打开
					}
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE); // 关闭
					DART_SET_BIT(2, FLAG_FIRED);
					i=0;
					DartSys.currentStep++;
				}
				break;
			}
			case 3:
			{
				setKey3Auto();
				if(!DART_CHECK_BIT(3,FLAG_DART_DROPPED))
					break;
				if (!DART_CHECK_BIT(3, FLAG_TRIGGER_AT_SHOOT_POS)) {
					DJIMotorSetRef(chargeLoader, PURPLE_25M_SHOOT_ANGLE);
					if(CHECK_ANGLE_ARRIVED(chargeLoader->measure.total_angle, PURPLE_25M_SHOOT_ANGLE,MOTOR_ANGLE_DEADBAND)) {// 到达位置后设置标志位
						DART_SET_BIT(3, FLAG_TRIGGER_AT_SHOOT_POS);
					}
					break;
				}
				if (DART_CHECK_MASK(3, MASK_READY_TO_FIRE)) {
					// 扳机打开，发射飞镖
					if(start_3_time == 0) {
						start_3_time = HAL_GetTick();
					}
					ServoSetAngle(banji_motor, BANJI_OPEN_ANGLE);
					uint32_t dt = HAL_GetTick() - start_3_time;
					if(dt < 1000) {
						break; // 延时200ms确保扳机打开
					}
					ServoSetAngle(banji_motor, BANJI_CLOSE_ANGLE); // 关闭
					DART_SET_BIT(3, FLAG_FIRED);
					i=0;
					DartSys.currentStep++;
				}
				tmpa=1;
				break;
			}
			default:
				break;
		}
		break;
	default:
		break;
	}
	
	PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
