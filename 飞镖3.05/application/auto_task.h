#ifndef AUTO_TASK_H
#define AUTO_TASK_H

#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "referee_usart_task.h"
#include "usb_task.h"
#include "voltage_task.h"
#include "servo_task.h"
#include "User_Task.h"
#include "cmsis_os.h"
#include "referee.h"
#include "protocol.h"

typedef struct{
	uint16_t data[10];
}Sensor_measure_t;

extern void auto_task(void const *pvParameters);

extern void auto_mode(ext_dart_info_t dart_info_t);//设置打击模式 哨塔or基地

extern Sensor_measure_t get_Ssensor_control(void);//根据s型传感器传输数据



#endif
