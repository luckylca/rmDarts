/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "gimbal_task.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

extern jpid frc1pid;
extern jpid frc2pid;
#define  frc1speed 42.0f  //42
#define  frc2speed 42.0f
int sj_flag=0;
static int cyy_flag; //用于切换飞镖发射位前进后退的


/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //射击数据


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
		// 设置拨盘速度的PID
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 SHOOT_PID[3] = {15.0f, 0.0f, 0.0f};
		// 初始化模式SHOOT_STOP
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针 kmk km
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
		//拨弹电机
    shoot_control.shoot_motor_measure[0] = get_trigger_motor_measure_point();
		//发射电机
		shoot_control.shoot_motor_measure[1] = get_frc1_motor_measure_point();
		shoot_control.shoot_motor_measure[2] = get_frc2_motor_measure_point();
    //初始化拨盘PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		//初始化发射PID
		PID_init(&shoot_control.shootpid,PID_POSITION,SHOOT_PID,10000,7000);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.lid_pwm = FRIC_OFF;
		shoot_control.lid_pwm2 = FRIC_OFF;	
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure[0]->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */

static int bppsan_init_flag=0;

void bopan_init()
{
        shoot_control.set_angle = -2.106569052;
	      shoot_control.speed_set = 70*rad_format(shoot_control.set_angle - shoot_control.angle);
}



// 循环内部计算输出shoot值
int16_t shoot_control_loop(void)
{
    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
	  bppsan_init_flag=1;      //拨盘初始化标志位为1
	
	  if(bppsan_init_flag == 0)
		{
			 bopan_init();
			 PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
			 shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
		}
		else
		{
			if (shoot_control.shoot_mode == SHOOT_STOP)
			{
					//设置拨弹轮的速度
					shoot_control.speed_set = 0.0f;
			}
			else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
			{
					//设置拨弹轮的速度
					shoot_control.speed_set = 0.0f;
			}
			else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
			{
					if(shoot_control.key == SWITCH_TRIGGER_OFF)
					{
							//设置拨弹轮的拨动速度,并开启堵转反转处理
							shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
							trigger_motor_turn_back();
					}
					else
					{
							shoot_control.trigger_speed_set = 0.0f;
							shoot_control.speed_set = 0.0f;
					}
					shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
					shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
			}
			else if (shoot_control.shoot_mode == SHOOT_READY)
			{
					//设置拨弹轮的速度
					 shoot_control.speed_set = 0.0f;
			}
			// 间断发射
			else if (shoot_control.shoot_mode == SHOOT_BULLET)
			{
					shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
					shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
					shoot_bullet_control();
			}
			// 连续发射
			else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
			{
					//设置拨弹轮的拨动速度,并开启堵转反转处理
					shoot_control.trigger_speed_set = -CONTINUE_TRIGGER_SPEED;
					trigger_motor_turn_back();
			}
			else if (shoot_control.shoot_mode == SHOOT_CYY_FORWARD)
			{
					// 向前（发射方向为前）
					shoot_control.trigger_speed_set = -CYY_CHANGE_SPEED_2;
					trigger_motor_turn_back();
			}
			else if (shoot_control.shoot_mode == SHOOT_CYY_BACK)
			{
					// 向后（发射方向为前）
					shoot_control.trigger_speed_set = CYY_CHANGE_SPEED_2;
					trigger_motor_turn_back();
			}
			else if(shoot_control.shoot_mode == SHOOT_DONE)
			{
					shoot_control.speed_set = 0.0f;
			}
			
			
			if(shoot_control.shoot_mode == SHOOT_STOP)
			{
					shoot_laser_off();
					shoot_control.given_current = 0;
					frc1pid.out=0.0f;
					frc2pid.out=0.0f;
					sj_flag=0;

			}
			else
			{
					shoot_laser_on(); //激光开启
					//计算拨弹轮电机PID
					PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
					shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
					if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
					{
							shoot_control.given_current = 0;
					}
					sj_flag=1;

			}
	  }
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
		//初始化时是stop
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
				cyy_flag=1;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
			  cyy_flag=0;
    }

		
    if(cyy_flag&&(switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])))
    {
        shoot_control.shoot_mode = SHOOT_CYY_FORWARD;
    }
    else if(cyy_flag && (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])))
    {
        shoot_control.shoot_mode = SHOOT_CYY_BACK;
    }
		
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure[0]->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

		//获得发射机构的旋转速度
		frc1pid.ref =(0.000415809748903494517209f * shoot_control.shoot_motor_measure[1]->speed_rpm);
		frc2pid.ref =(0.000415809748903494517209f * shoot_control.shoot_motor_measure[2]->speed_rpm);
		frc1pid.set=  -2.21;//-frc1speed/20.0f;         //42/19?
		frc2pid.set= 2.21;  //frc2speed/20.0f;
		frc1pid.out=0.0f;
		frc2pid.out=0.0f;
    //chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		
    //拨盘电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure[0]->ecd - shoot_control.shoot_motor_measure[0]->last_ecd > HALF_ECD_RANGE)
    {
		//如果此次数与上次之差大于半圈就减一
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure[0]->ecd - shoot_control.shoot_motor_measure[0]->last_ecd < -HALF_ECD_RANGE)
    {
		//如果此次数与上次之差小于负的半圈就加一
        shoot_control.ecd_count++;
    }
		
		// 如果编码器计数值满情况判断+-18
    if (shoot_control.ecd_count == FULL_COUNT)
    {	
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    // 计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure[0]->ecd) * MOTOR_ECD_TO_ANGLE;
}


// 用于处理堵转情况的函数
static void trigger_motor_turn_back(void)
{
	
    if( shoot_control.block_time < BLOCK_TIME)
    {
		// 如果没有堵转
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
		// 如果堵转，反向旋转
        shoot_control.speed_set = 6.5;
    }

		
		//reverse_time应该是反向旋转的时间
    if( shoot_control.block_time < BLOCK_TIME )
    {
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time >= BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
			//如果反向旋转的时间没有超过最大值就一直累加
        shoot_control.reverse_time++;
    }
    else
    {
			//反向旋转的时间到达最大后，将堵转时间归零
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
	
	     static int n=0;
			// 旋转标志为0时
	     if(shoot_control.move_flag == 0)
	     { 
					 // 设定角度等于当前角度减去二分之派
					 shoot_control.set_angle = rad_format(shoot_control.angle - PI_FOUR*2);
					 // 如果设定角度的绝对值大于派减十分之派的绝对值 则 设定角度设置为这个绝对值
					 if(shoot_control.set_angle <= -PI_FOUR*4 + PI_TEN || shoot_control.set_angle >= PI_FOUR*4 - PI_TEN )
					 {
							shoot_control.set_angle = PI_FOUR*4 - PI_TEN;
					 }
					 // 旋转标志位启动
					 shoot_control.move_flag = 1;
					 // 堵转时间初始化
					 shoot_control.block_time = 0;
			 }
			 
			 // 如果到达设定角度附件，成功运行时间++，堵转时间清零
			 if(rad_format(shoot_control.set_angle - shoot_control.angle)  < 0.05f && rad_format(shoot_control.set_angle - shoot_control.angle) > -0.05f)
			 {
					 shoot_control.ac_time++;
					 shoot_control.block_time = 0;
			 }
			 else
			 {
			// 如果没有达到设定角度附近，堵转时间累加，成功运行时间清零
					 shoot_control.ac_time=0;
					 shoot_control.block_time++;
       }
			 // 拨盘pid，这里使用了等价的条件，虽然目标设为0，但是输入函数的当前值其实就是差值
			 shoot_control.trigger_speed_set = PID_calc(&(shoot_control.shootpid) , rad_format(shoot_control.angle - shoot_control.set_angle) , 0);
			 // 调用堵转处理函数
       trigger_motor_turn_back();
			 // 如果正常运行时间达到500则移动标志位 置0 射击模式设置为射击完成
			 if(shoot_control.ac_time>=500)
			 {
					 shoot_control.move_flag = 0;
					 shoot_control.shoot_mode = SHOOT_DONE;
			 }

}

