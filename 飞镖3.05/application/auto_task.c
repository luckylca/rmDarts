#include "auto_task.h"
#include "cmsis_os.h"
#include "referee.h"
#include "protocol.h"
#include "servo_task.h"
#include "bsp_servo_pwm.h"


#define PULL_DOWN_POWER 100 //下拉
#define PULL_DOW_TMIE 5 
#define PULL_UP_POWER 50   //上拉
#define PULL_UP_TIME 10 
#define ROLL_POWER 10    //换弹
#define ROLL_TIME 100  
#define TRIGGER_POWER 1000 //舵机
#define LOC_CHANGE_POWER 100 //丝杆
#define MAX_POWER  1500 //下拉最大力
#define MIN_POWER  10 //上拉最小力
#define ROTATE_POWER 100  //旋转
#define ROTATE_TIME  10 

uint8_t shoot_flag = 1;
uint8_t step=0;
ext_dart_info_t dart_info_r;
ext_dart_client_cmd_t dart_client_cmd_r;

uint16_t power;

Sensor_measure_t get_Ssensor_control(void)
{
	
}


void auto_mode(ext_dart_info_t dart_info_t)
{
	uint8_t latest_aim=(uint8_t)(dart_info_r.dart_info[0]|dart_info_r.dart_info[1]);
	uint8_t latest_count=(uint8_t)(dart_info_r.dart_info[2]|dart_info_r.dart_info[3]|dart_info_r.dart_info[4]);
	uint8_t now_aim=(uint8_t)(dart_info_r.dart_info[5]|dart_info_r.dart_info[6]);
	if(latest_aim==0&&now_aim==1)
	{
		for(int i=0 ;i < 100 ;i++)
		{
				CAN_cmd_gimbal(ROTATE_POWER,0,0, 0);
		}
		for(int i=0 ;i < 100 ;i++)
		{
			CAN_cmd_gimbal_frc(LOC_CHANGE_POWER ,0, 0);
		}
	}
	else if(latest_aim==1&&now_aim==0)
	{
		for(int i=0 ;i < 100 ;i++)
		{
				CAN_cmd_gimbal(-ROTATE_POWER,0,0, 0);
		}
		for(int i=0 ;i < 100 ;i++)
		{
			CAN_cmd_gimbal_frc(-LOC_CHANGE_POWER ,0, 0);
		}
	}
	
}

void auto_task(void const *pvParameters)
{
	  vTaskDelay(CHASSIS_TASK_INIT_TIME);
	  uint8_t cnt=0;
	  while(1)
		{
			 auto_mode(dart_info_t);
			if(dart_client_cmd_r.dart_launch_opening_stauts==0)//开启
			{
      	if(shoot_flag == 1 && step == 0)//发射
				{
					servo_pwm_set(TRIGGER_POWER, 3);
					shoot_flag = 0;
					step++;
				}
				else if(step == 1 && shoot_flag ==0 && power <= MAX_POWER)//下拉
				{
					CAN_cmd_chassis(PULL_DOWN_POWER,PULL_DOWN_POWER,0,0);
					if(power == MAX_POWER)
					{
						step++;
					}
				}
				else if(step == 2 && shoot_flag ==0)//换弹
				{
			    for(int i=0; i < 100; i++)
					    CAN_cmd_gimbal(0, 0, ROLL_POWER, 0);
					step++;
				}
				else if(step == 3 && shoot_flag == 0 && power >= MIN_POWER)//上拉
				{
					CAN_cmd_chassis(PULL_UP_POWER,PULL_UP_POWER,0,0);
				}
				cnt++;
				if(cnt==4||dart_client_cmd_r.dart_launch_opening_stauts==1 || dart_client_cmd_r.dart_launch_opening_stauts==2 ||dart_info_r.dart_remaining_time==0)
				{
						break;
				}				
			}
		}
	  
}

