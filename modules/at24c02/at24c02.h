#ifndef AT24C02
#define AT24C02

#include "gpio.h"
#include "main.h"
#include <stdint.h>

#define motorTotalAngleSetCNT 10

#define AT24C02_WADDR 0xA0
#define AT24C02_RADDR 0xA1

// 定义 I2C 的 SDA 和 SCL 引脚
#define IIC_SCL_GPIO_PORT GPIOI
#define IIC_SCL_PIN GPIO_PIN_6
#define IIC_SDA_GPIO_PORT GPIOI
#define IIC_SDA_PIN GPIO_PIN_7

// 定义 GPIO 操作的宏
#define IIC_SCL_HIGH() HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_LOW() HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define IIC_SDA_HIGH() HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_LOW() HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define IIC_SDA_READ() HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN)

// 外部全局数组声明
extern int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT];
extern int32_t DmMotorTotalAngleSet[motorTotalAngleSetCNT];

// AT24C02 读写接口函数声明
uint8_t AT24C02_ReadByte(uint16_t addr);
void AT24C02_WriteByte(uint16_t addr, uint8_t data);
void AT24C02_Read(uint16_t addr, uint8_t *buffer, uint16_t length);
void AT24C02_Write(uint16_t addr, uint8_t *buffer, uint16_t length);

// 读写所有电机总角度数据
// DJI 电机: 地址 0x00-0x27 (前 10 个 int32_t)
// DM 电机:  地址 0x28-0x4F (后 10 个 int32_t)
void readAllMotorTotalAngleSet(void);
void writeAllMotorTotalAngleSet(void);

// DJI 电机总角度接口 (地址 0x00-0x27)
int32_t readDjiMotorTotalAngleSetByIndex(uint8_t idx);
void writeDjiMotorTotalAngleSetByIndex(uint8_t idx, int32_t value);

// DM 电机总角度接口 (地址 0x28-0x4F)
int32_t readDmMotorTotalAngleSetByIndex(uint8_t idx);
void writeDmMotorTotalAngleSetByIndex(uint8_t idx, int32_t value);
#endif // AT24C02