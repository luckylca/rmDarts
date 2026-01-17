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

// int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT] = {0};

#define AT24C02_WADDR 0xA0
#define AT24C02_RADDR 0xA1

void AT24C02_WriteByte(uint16_t addr, uint8_t wdata)
{
	HAL_I2C_Mem_Write(&hi2c2, AT24C02_WADDR, addr, I2C_MEMADD_SIZE_8BIT, &(wdata), 1, 1000);
}

void AT24C02_WritePage(uint16_t addr, uint8_t *pdata)
{
	HAL_I2C_Mem_Write(&hi2c2, AT24C02_WADDR, addr, I2C_MEMADD_SIZE_8BIT, pdata, 8, 1000);
}

uint8_t AT24C02_ReadByte(uint16_t addr)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c2, AT24C02_RADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	return data;
}

void AT24C02_ReadPage(uint16_t addr, uint8_t *pdata)
{
	HAL_I2C_Mem_Read(&hi2c2, AT24C02_RADDR, addr, I2C_MEMADD_SIZE_8BIT, pdata, 8, 1000);
}

HAL_StatusTypeDef AT24C02_ReadData(uint16_t addr, uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, AT24C02_WADDR, addr, I2C_MEMADD_SIZE_8BIT, pData, size, 1000);
	// HAL_Delay(5);
	return status;
}

HAL_StatusTypeDef AT24C02_WriteData(uint16_t addr, uint8_t *pdata, uint16_t size)
{
	HAL_StatusTypeDef status;
	uint8_t write_len;
	uint8_t page_remain = 8 - (addr % 8); // 当前页还剩多少字节

	while (size > 0)
	{
		// 本次写入的长度取 (剩余长度) 和 (当前页剩余空间) 的较小值
		write_len = (size > page_remain) ? page_remain : size;

		status = HAL_I2C_Mem_Write(&hi2c2, AT24C02_WADDR, addr,
								   I2C_MEMADD_SIZE_8BIT, pdata, write_len, 1000);

		if (status != HAL_OK)
			return status;

		// 【关键】物理写入等待
		// 在 RTOS 中请务必使用 osDelay(5)，非 RTOS 使用 HAL_Delay(5)
		osDelay(5);

		// 更新参数
		size -= write_len;	// 待写长度减少
		pdata += write_len; // 数据指针后移
		addr += write_len;	// 目标地址后移
		page_remain = 8;	// 之后的写入都从新页开头开始，所以剩余空间直接设为8
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

    for (uint8_t i = 0; i < idx; i++)
    {
        // 1. 从 EEPROM 读取 4 字节并恢复为 float
        // 我们之前分配的地址是 idx * 4
        AT24C02_ReadData(i * 4, (uint8_t *)&temp_angle, sizeof(float));

        // 2. 赋值给实时数据结构体（用于控制逻辑的起始点）
        // 假设你的结构体里有对应的角度变量
        GenericMotorData_s *motor_data = motor_recoder_data[i];
        switch (motor_data->type)
        {
            case DJI_MOTOR: motor_data->data.dji.angle = temp_angle; motor_recoder_last_data[i]->data.dji.angle = temp_angle; break;
            case DM_MOTOR:  motor_data->data.dm.angle  = temp_angle; motor_recoder_last_data[i]->data.dm.angle = temp_angle; break;
            case HT_MOTOR:  motor_data->data.ht.angle  = temp_angle; motor_recoder_last_data[i]->data.ht.angle = temp_angle; break;
            case JZ_MOTOR:  motor_data->data.jz.angle  = temp_angle; motor_recoder_last_data[i]->data.jz.angle = temp_angle; break;
            case LK_MOTOR:  motor_data->data.lk.angle  = temp_angle; motor_recoder_last_data[i]->data.lk.angle = temp_angle; break;
            default: break;
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

void RecodeAngleTask()
{
	for (uint8_t i = 0; i < idx; i++)
	{
		GenericMotorData_s *motor_data = motor_recoder_data[i];
		switch (motor_data->type)
		{
		case DJI_MOTOR:
			motor_data->data.dji.angle = motor_data->data.dji.dji_motor->measure.total_angle;
			if(fabsf(motor_data->data.dji.angle-motor_recoder_last_data[i]->data.dji.angle) >5.0f)
			{
				AT24C02_WriteData(i * 4, (uint8_t *)&(motor_data->data.dji.angle), sizeof(float));
				motor_recoder_last_data[i]->data.dji.angle = motor_data->data.dji.angle;
			}
			break;
		case DM_MOTOR:
			// motor_data->data.dm.angle = motor_data->data.dm.dm_motor->measure.total_angle;
			// if(fabsf(motor_data->data.dm.angle-motor_recoder_last_data[i]->data.dm.angle) >5.0f)
			// {
			// 	AT24C02_WriteData(i * 4, (uint8_t *)&(motor_data->data.dm.angle), sizeof(float));
			// 	motor_recoder_last_data[i]->data.dm.angle = motor_data->data.dm.angle;
			// }
			break;
		case HT_MOTOR:
			motor_data->data.ht.angle = motor_data->data.ht.ht_motor->measure.total_angle;
			if(fabsf(motor_data->data.ht.angle-motor_recoder_last_data[i]->data.ht.angle) >5.0f)
			{
				 AT24C02_WriteData(i * 4, (uint8_t *)&(motor_data->data.ht.angle), sizeof(float));
				motor_recoder_last_data[i]->data.ht.angle = motor_data->data.ht.angle;
			}
			break;
		case JZ_MOTOR:
			motor_data->data.jz.angle = motor_data->data.jz.jz_motor->measure.total_angle;
			if(fabsf(motor_data->data.jz.angle-motor_recoder_last_data[i]->data.jz.angle) >5.0f)
			{
				AT24C02_WriteData(i * 4, (uint8_t *)&(motor_data->data.jz.angle), sizeof(float));
				motor_recoder_last_data[i]->data.jz.angle = motor_data->data.jz.angle;
			}
			break;
		case LK_MOTOR:
			motor_data->data.lk.angle = motor_data->data.lk.lk_motor->measure.total_angle;
			if(fabsf(motor_data->data.lk.angle-motor_recoder_last_data[i]->data.lk.angle) >5.0f)
			{
				AT24C02_WriteData(i * 4, (uint8_t *)&(motor_data->data.lk.angle), sizeof(float));
				motor_recoder_last_data[i]->data.lk.angle = motor_data->data.lk.angle;
			}
			break;
		default:
			break;
		}
	}
}