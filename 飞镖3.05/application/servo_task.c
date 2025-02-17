/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"


const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
int16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MAX_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
int i =0;
//extern TIM_HandleTypeDef htim8;
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
// 飞镖释放函数
static void DART_RELEASE_FUNCTION(void)  
{			
	// 设置pwm
			servo_pwm[0]=DART_RELEASE;
      servo_pwm_set(servo_pwm[0], 3);
			osDelay(500);
}


void servo_task(void const * argument)
{
	// 获取遥控器指针
    servo_rc = get_remote_control_point();
	// 循环读取遥控器信息
    while(1)
    {
			// 左侧拨轮下拨则发射
			if(servo_rc->rc.ch[4] > 0)
			{
				DART_RELEASE_FUNCTION();
			}	
			// 默认锁定状态
			servo_pwm[0]=DART_HOLD;
			servo_pwm_set(servo_pwm[0], 3);
			
      osDelay(10);
    }
}
