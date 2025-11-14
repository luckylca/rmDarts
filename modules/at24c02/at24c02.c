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

int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT] = {0};

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
	uint16_t page_count = (size + 7) / 8;
	
	for (uint16_t i = 0; i < page_count; i++)
	{
		uint16_t write_addr = addr + i * 8;
		uint8_t write_len = (size - i * 8) > 8 ? 8 : (size - i * 8);
		
		status = HAL_I2C_Mem_Write(&hi2c2, AT24C02_WADDR, write_addr, I2C_MEMADD_SIZE_8BIT, 
								   pdata + i * 8, write_len, 1000);
		
		if (status != HAL_OK)
		{
			return status;
		}
		
		// HAL_Delay(5);
	}
	
	return HAL_OK;
}


void readAllMotorAngle()
{
	memset(&DjiMotorTotalAngleSet, 0, motorTotalAngleSetCNT * sizeof(int32_t));
	AT24C02_ReadData(0, (uint8_t *)&DjiMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
}

void writeDjiMotorAngle(uint8_t idx, int32_t angle)
{
	uint8_t buf[4];
	buf[0] = (uint8_t)((angle >> 0) & 0xFF);
	buf[1] = (uint8_t)((angle >> 8) & 0xFF);
	buf[2] = (uint8_t)((angle >> 16) & 0xFF);
	buf[3] = (uint8_t)((angle >> 24) & 0xFF);

	AT24C02_WriteData(idx * 4, buf, 4);
}

void setAllMotorZero()
{
	uint8_t zero_buf[8] = {0};
	for (uint16_t i = 0; i < 32; i++)
	{
		AT24C02_WritePage(i * 8, zero_buf);
	}
}