#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"

#define SERVO_MIN_PWM   -1000
#define SERVO_MAX_PWM   1425
#define SERVO1_MIN_PWM_1   -1000
#define SERVO1_MAX_PWM_2   1425
#define SERVO1_MIN_PWM_3   -1000
#define SERVO1_MAX_PWM_4   1425

#define SERVO2_MIN_PWM_1   -1000
#define SERVO2_MAX_PWM_2   1425
#define SERVO2_MIN_PWM_3   -1000
#define SERVO2_MAX_PWM_4   1425

#define SERVO3_MIN_PWM_1   -1000
#define SERVO3_MAX_PWM_2   1425
#define SERVO3_MIN_PWM_3   -1000
#define SERVO3_MAX_PWM_4   1425

#define DART_RELEASE   1500 //飞镖释放值
#define DART_HOLD   1300  // 飞镖锁定值


#define PWM_DETAL_VALUE 500
#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT


extern void servo_task(void const * argument);

#endif
