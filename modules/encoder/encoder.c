/**
 * @file encoder.c
 * @author lucky
 * @author lca
 * @version 1.0
 * @date 2025-11-14
 *
 * @copyright Copyright (c) lucky all rights reserved
 *
 */
#include "encoder.h"

//-------------------------------------------------------------------------------------------------------------------
//  @brief      通过SPI写一个byte,同时读取一个byte
//  @param      byte        发送的数据
//  @return     uint8_t       return 返回status状态
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static uint8_t spi_wr_byte(uint8_t byte)
{
    HAL_SPI_TransmitReceive(&hspi2, &byte, &byte, 1, 0xFFFF);
    return (byte);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将val写入cmd对应的寄存器地址,同时返回status字节
//  @param      cmd         命令字
//  @param      val         待写入寄存器的数值
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void spi_w_reg_byte(uint8_t cmd, uint8_t val)
{
    ABS_ENCODER_CSN(0);
    cmd |= ABS_ENCODER_SPI_W;
    spi_wr_byte(cmd);
    spi_wr_byte(val);
    ABS_ENCODER_CSN(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取cmd所对应的寄存器地址
//  @param      cmd         命令字
//  @param      *val        存储读取的数据地址
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void spi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    ABS_ENCODER_CSN(0);
    cmd |= ABS_ENCODER_SPI_R;
    spi_wr_byte(cmd);
    *val = spi_wr_byte(0);
    ABS_ENCODER_CSN(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      写入一个数据到编码器的寄存器
//  @param      cmd         寄存器地址
//  @param      *val        写入数据的地址
//  @return     uint8_t       0：程序  1：失败
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static uint8_t encoder_spi_w_reg_byte(uint8_t cmd, uint8_t val)
{
    uint8_t dat;
    ABS_ENCODER_CSN(0);
    cmd |= ABS_ENCODER_SPI_W;
    spi_wr_byte(cmd);
    spi_wr_byte(val);
    ABS_ENCODER_CSN(1);
    DWT_Delay(0.01);
    ABS_ENCODER_CSN(0);
    dat = spi_wr_byte(0x00);
    spi_wr_byte(0x00);
    ABS_ENCODER_CSN(1);

    if (val != dat)
        return 1; // 写入失败
    return 0;     // 写入成功
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      读取寄存器
//  @param      cmd         寄存器地址
//  @param      *val        存储读取的数据地址
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void encoder_spi_r_reg_byte(uint8_t cmd, uint8_t *val)
{
    ABS_ENCODER_CSN(0);
    cmd |= ABS_ENCODER_SPI_R;
    spi_wr_byte(cmd);
    spi_wr_byte(0x00);

    ABS_ENCODER_CSN(1);
    DWT_Delay(0.001);
    ABS_ENCODER_CSN(0);
    *val = spi_wr_byte(0x00);
    spi_wr_byte(0x00);
    ABS_ENCODER_CSN(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置零偏
//  @param      zero_position  需要设置的零偏
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static void set_zero_position_spi(uint16_t zero_position)
{
    zero_position = (uint16_t)(4096 - zero_position);
    zero_position = zero_position << 4;
    encoder_spi_w_reg_byte(ZERO_L_REG, (uint8_t)zero_position); // 设置零位
    encoder_spi_w_reg_byte(ZERO_H_REG, zero_position >> 8);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      写入一个数据到编码器的寄存器
//  @param      void
//  @return     uint16_t       返回角度值0-4096 对应0-360°
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

uint16_t encoder_angle_spi(void)
{
    uint16_t angle;
    angle = 0;
    ABS_ENCODER_CSN(0);
    angle = (uint16_t)spi_wr_byte(0x00);
    angle <<= 8;                          // 存储高八位
    angle |= (uint16_t)spi_wr_byte(0x00); // 存储低八位
    ABS_ENCODER_CSN(1);

    return (angle >> 4); // 12位精度，因此右移四位
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器自检函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void encoder_self5_check(void)
{
    uint8_t val;
    do
    {
        encoder_spi_r_reg_byte(6, &val);
        // 卡在这里原因有以下几点
        // 1 编码器坏了，如果是新的这样的概率极低
        // 2 接线错误或者没有接好
    } while (0x1C != val);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器初始化函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void encoder_init_spi(void)
{
    encoder_self5_check();                 // debug 暂时注释掉自检
    encoder_spi_w_reg_byte(DIR_REG, 0x00); // 设置旋转方向 正转数值变小：0x00   反转数值变大：0x80
    set_zero_position_spi(0);              // 设置零偏
}

static EncoderInstance *encoder_instance = NULL;
EncoderInstance *EncoderInit(SPI_Init_Config_s *spi_config)
{
    // SPIInstance *spi_instance = SPIRegister(spi_config);

    EncoderInstance *encoder = malloc(sizeof(EncoderInstance));
    memset(encoder, 0, sizeof(EncoderInstance));

    encoder->measure.angle = 0;
    encoder->measure.position = 0;
    // encoder->encoder_spi_instance = spi_instance;

    encoder_init_spi();

    /* 保存为模块静态实例，供 EncoderTask 等函数使用 */
    encoder_instance = encoder;

    return encoder;
}

void EncoderTask()
{
    encoder_instance->measure.position = encoder_angle_spi();
    encoder_instance->measure.angle = encoder_instance->measure.position *ZF_ENCODER_ECD_TO_DEGREE;
    // 读取编码器数据的任务实现
}