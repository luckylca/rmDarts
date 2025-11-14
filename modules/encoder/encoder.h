#ifndef ENCODER_H
#define ENCODER_H
#include "bsp_spi.h"
#include <string.h>
#include <stdlib.h>

#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define ABS_ENCODER_SPI_CSN(x) HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin,(GPIO_PinState)x)


//角度传感器参数
#define ABS_ENCODER_SPI_W       0x80
#define ABS_ENCODER_SPI_R       0x40


#define ZERO_L_REG              0x00
#define ZERO_H_REG              0x01
#define DIR_REG                 0X09

void   encoder_init_spi(void);
uint16_t encoder_angle_spi(void);

typedef struct EncoderInstance {
    int position;
    SPIInstance *encoder_spi_instance;
} EncoderInstance;

EncoderInstance *EncoderInit(SPI_Init_Config_s *spi_config);
void EncoderTask();
#endif