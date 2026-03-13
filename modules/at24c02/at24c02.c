/**
 * @file at24c02.c
 * @author lucky
 * @author lca
 * @version 1.0
 * @date 2025-11-14
 *
 * @copyright Copyright (c) lucky all rights reserved
 *
 */
#include "at24c02.h"
#include "i2c.h"
#include "string.h"
#include "FreeRTOS.h"

// int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT] = {0};

#define AT24C02_WADDR 0xA0
#define AT24C02_RADDR 0xA1
#define AT24C02_DEV_ADDR (0x50 << 1)
#define AT24C02_PAGE_SIZE 8U
#define AT24C02_SIZE_BYTES 256U
#define AT24C02_READY_TRIALS 20U
#define AT24C02_READY_TIMEOUT_MS 10U
#define AT24C02_RW_RETRY 3U

static HAL_StatusTypeDef AT24C02_WaitReady(void)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, AT24C02_DEV_ADDR, AT24C02_READY_TRIALS, AT24C02_READY_TIMEOUT_MS);
}

void AT24C02_WriteByte(uint16_t addr, uint8_t wdata)
{
    HAL_I2C_Mem_Write(&hi2c2, AT24C02_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &(wdata), 1, 1000);
}

void AT24C02_WritePage(uint16_t addr, uint8_t *pdata)
{
    HAL_I2C_Mem_Write(&hi2c2, AT24C02_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, pdata, 8, 1000);
}

uint8_t AT24C02_ReadByte(uint16_t addr)
{
	uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c2, AT24C02_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	return data;
}

void AT24C02_ReadPage(uint16_t addr, uint8_t *pdata)
{
    HAL_I2C_Mem_Read(&hi2c2, AT24C02_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, pdata, 8, 1000);
}

HAL_StatusTypeDef AT24C02_ReadData(uint16_t addr, uint8_t *pData, uint16_t size)
{
    if (pData == NULL || size == 0)
    {
        return HAL_ERROR;
    }

    if ((uint32_t)addr + (uint32_t)size > AT24C02_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    if (AT24C02_WaitReady() != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(&hi2c2, AT24C02_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, pData, size, 1000);
}

HAL_StatusTypeDef AT24C02_WriteData(uint16_t addr, uint8_t *pdata, uint16_t size)
{
    if (pdata == NULL || size == 0)
    {
        return HAL_ERROR;
    }

    if ((uint32_t)addr + (uint32_t)size > AT24C02_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    uint16_t offset = 0;
    while (offset < size)
    {
        uint16_t write_addr = addr + offset;
        uint8_t remain_in_page = (uint8_t)(AT24C02_PAGE_SIZE - (write_addr % AT24C02_PAGE_SIZE));
        uint16_t remain_total = (uint16_t)(size - offset);
        uint8_t write_len = (remain_total < remain_in_page) ? (uint8_t)remain_total : remain_in_page;

        HAL_StatusTypeDef status = HAL_ERROR;
        for (uint8_t retry = 0; retry < AT24C02_RW_RETRY; retry++)
        {
            if (AT24C02_WaitReady() != HAL_OK)
            {
                continue;
            }

            status = HAL_I2C_Mem_Write(&hi2c2,
                                   AT24C02_DEV_ADDR,
                                   write_addr,
                                   I2C_MEMADD_SIZE_8BIT,
                                   pdata + offset,
                                   write_len,
                                   1000);
            if (status == HAL_OK && AT24C02_WaitReady() == HAL_OK)
            {
                break;
            }
        }

        if (status != HAL_OK)
        {
            return status;
        }

        offset += write_len;
    }

    return HAL_OK;
}

static uint8_t idx;
static GenericMotorData_s *motor_recoder_data[motorTotalAngleSetCNT] = {NULL};
static GenericMotorData_s *motor_recoder_last_data[motorTotalAngleSetCNT] = {NULL};
// void readAllMotorAngle()
// {
// 	memset(&DjiMotorTotalAngleSet, 0, motorTotalAngleSetCNT * sizeof(int32_t));
// 	AT24C02_ReadData(0, (uint8_t *)&DjiMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
// }

// void writeDjiMotorAngle(uint8_t idx, int32_t angle)
// {
// 	uint8_t buf[4];
// 	buf[0] = (uint8_t)((angle >> 0) & 0xFF);
// 	buf[1] = (uint8_t)((angle >> 8) & 0xFF);
// 	buf[2] = (uint8_t)((angle >> 16) & 0xFF);
// 	buf[3] = (uint8_t)((angle >> 24) & 0xFF);

// 	AT24C02_WriteData(idx * 4, buf, 4);
// }

void setAllMotorZero()
{
	uint8_t zero_buf[8] = {0};
	for (uint16_t i = 0; i < 32; i++)
	{
		AT24C02_WritePage(i * 8, zero_buf);
	}
}

void motorDataInit()
{
    float temp_angle = 0.0f;
    HAL_StatusTypeDef read_status;

    for (uint8_t i = 0; i < idx; i++)
    {
        temp_angle = 0.0f;
        // 1. 从 EEPROM 读取 4 字节并恢复为 float
        // 我们之前分配的地址是 idx * 4
        read_status = AT24C02_ReadData(i * 4, (uint8_t *)&temp_angle, sizeof(float));

        if (read_status != HAL_OK) {
            temp_angle = 0.0f;
        }

        // 检查 EEPROM 是否未初始化 (0xFFFFFFFF 对应 float NaN)
        if (isnan(temp_angle)) {
            temp_angle = 0.0f;
        }

        // 2. 赋值给实时数据结构体（用于控制逻辑的起始点）
        // 假设你的结构体里有对应的角度变量
        GenericMotorData_s *motor_data = motor_recoder_data[i];
        switch (motor_data->type)
        {
        case DJI_MOTOR:
            motor_data->data.dji.angle = temp_angle;
            motor_recoder_last_data[i]->data.dji.angle = temp_angle;
            if (motor_data->data.dji.dji_motor)
            {
                motor_data->data.dji.dji_motor->measure.total_angle = temp_angle;
                // 同时恢复圈数，防止CAN回调计算出错
                motor_data->data.dji.dji_motor->measure.total_round = (int32_t)(temp_angle / 360.0f);
            }
            break;
        case DM_MOTOR:
            motor_data->data.dm.angle = temp_angle;
            motor_recoder_last_data[i]->data.dm.angle = temp_angle;
            // 达妙电机根据具体情况恢复，这里假设恢复到 position 字段可能不够，需视达妙驱动实现而定
             if(motor_data->data.dm.dm_motor)
             {
                 // 注意：达妙电机通常是位置环，根据驱动不同可能需要不同处理
                 // 这里仅作示例，如果达妙驱动没有 total_angle 字段请自行确认
                 // motor_data->data.dm.dm_motor->measure.position = temp_angle; 
             }
            break;
        case HT_MOTOR:
            // motor_data->data.ht.angle = temp_angle;
            // motor_recoder_last_data[i]->data.ht.angle = temp_angle;
            //  if(motor_data->data.ht.ht_motor)
            //  {
            //      motor_data->data.ht.ht_motor->measure.total_angle = temp_angle;
            //      motor_data->data.ht.ht_motor->measure.total_round = (int32_t)(temp_angle / 360.0f);
            //  }
            break;
        case JZ_MOTOR:
            motor_data->data.jz.angle = temp_angle;
            motor_recoder_last_data[i]->data.jz.angle = temp_angle;
            break;
        case LK_MOTOR:
            motor_data->data.lk.angle = temp_angle;
            motor_recoder_last_data[i]->data.lk.angle = temp_angle;
            break;
        default:
            break;
        }

    }
}

void motorRecoderRegister(Motor_Recoder_Init_Config_s *config)
{
	motor_recoder_data[idx] = (GenericMotorData_s *)malloc(sizeof(GenericMotorData_s));
	motor_recoder_last_data[idx] = (GenericMotorData_s *)malloc(sizeof(GenericMotorData_s));
	motor_recoder_data[idx]->type = config->type;
	switch (config->type)
	{
	case DJI_MOTOR:
		motor_recoder_data[idx]->data.dji.dji_motor = config->data.dji;
		break;
	case DM_MOTOR:
		// motor_recoder_data[idx]->data.dm.dm_motor = config->data.dm;
		break;
	case HT_MOTOR:
		motor_recoder_data[idx]->data.ht.ht_motor = config->data.ht;
		break;
	case JZ_MOTOR:
		motor_recoder_data[idx]->data.jz.jz_motor = config->data.jz;
		break;
	case LK_MOTOR:
		motor_recoder_data[idx]->data.lk.lk_motor = config->data.lk;
		break;
	default:
		break;
	}
	idx++;
}
#define MIN_SAVE_DIFF       10.0f   // 角度变化超过该阈值才写
#define SAVE_MIN_INTERVAL_MS 500U  // 最小写入间隔，避免频繁擦写
void RecodeAngleTask(void)
{
    static uint32_t last_save_tick[motorTotalAngleSetCNT] = {0};
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < idx; i++)
    {
        GenericMotorData_s *motor = motor_recoder_data[i];
        GenericMotorData_s *last_saved = motor_recoder_last_data[i];
        float current_angle = 0.0f;
        float *saved_angle_ptr = NULL;

        // --- A. 获取当前角度 ---
        switch (motor->type)
        {
        case DJI_MOTOR:
            current_angle = motor->data.dji.dji_motor->measure.total_angle;
            saved_angle_ptr = &(last_saved->data.dji.angle);
            // 更新实时数据结构体缓存
            motor->data.dji.angle = current_angle;
            break;
        case HT_MOTOR:
            current_angle = motor->data.ht.ht_motor->measure.total_angle;
            saved_angle_ptr = &(last_saved->data.ht.angle);
            motor->data.ht.angle = current_angle;
            break;
        case DM_MOTOR:
            // 假设达妙电机使用 position 作为角度（弧度或度，需根据驱动确认）
            // 如果达妙是弧度制，这里可能需要转换
            // current_angle = motor->data.dm.dm_motor->measure.position; 
            saved_angle_ptr = &(last_saved->data.dm.angle);
            // motor->data.dm.angle = current_angle;
            break;
            // ... (其他电机 case) ...
        default:
            continue;
        }

        // 超过阈值并满足最小写入间隔即落盘，避免“必须静止”导致永不保存
        if (saved_angle_ptr != NULL &&
            fabsf(current_angle - *saved_angle_ptr) > MIN_SAVE_DIFF &&
            (now - last_save_tick[i]) >= SAVE_MIN_INTERVAL_MS)
        {
            if (AT24C02_WriteData(i * 4, (uint8_t *)&current_angle, sizeof(float)) == HAL_OK)
            {
                *saved_angle_ptr = current_angle; 
                last_save_tick[i] = now;
            }
        }
    }
}