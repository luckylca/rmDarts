#ifndef ENCODER_H
#define ENCODER_H
#include "bsp_spi.h"
#include <string.h>
#include <stdlib.h>

#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOB
#define ABS_ENCODER_SCK(x) HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, (GPIO_PinState)x)
#define ABS_ENCODER_MISO HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin)
#define ABS_ENCODER_MOSI(x) HAL_GPIO_WritePin(MOSI_GPIO_Port, MOSI_Pin, (GPIO_PinState)x)
#define ABS_ENCODER_CSN(x) HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, (GPIO_PinState)x)



//角度传感器参数
#define ABS_ENCODER_SPI_W       0x80
#define ABS_ENCODER_SPI_R       0x40


#define ZERO_L_REG              0x00
#define ZERO_H_REG              0x01
#define DIR_REG                 0X09

void Soft_SPI_GPIO_Init(void);
void   encoder_init_spi(void);
uint16_t encoder_angle_spi(void);

typedef struct EncoderInstance {
    int position;
    SPIInstance *encoder_spi_instance;
} EncoderInstance;

EncoderInstance *EncoderInit(SPI_Init_Config_s *spi_config);
void EncoderTask();
#endif