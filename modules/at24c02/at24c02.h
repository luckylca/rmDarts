#ifndef AT24C02
#define AT24C02

#include "main.h"
#include <stdint.h>

#define motorTotalAngleSetCNT 20

#define DJI_MOTOR_ANGLE_CNT 15
#define DM_MOTOR_ANGLE_CNT 5

#define AT24C02_WADDR 0xA0  // AT24C02 写地址
#define AT24C02_RADDR 0xA1  // AT24C02 读地址

// 外部全局数组声明
extern int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT];
extern int32_t DmMotorTotalAngleSet[motorTotalAngleSetCNT];

void readAllMotorAngle();

void writeDjiMotorAngle(uint8_t idx,int32_t angle);

void setAllMotorZero();
#endif // AT24C02
