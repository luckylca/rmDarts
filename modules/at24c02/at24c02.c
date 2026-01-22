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
#define RECODE_PERIOD_MS   100   // 该函数的调用周期 (必须与 osDelay 一致)
#define IDLE_TIMEOUT_MS    2000  // 静止多久才保存 (2秒)
#define MIN_SAVE_DIFF      5.0f  // 只有变化超过5度才写 EEPROM
#define IDLE_CHECK_DIFF    0.5f  // 判定是否静止的抖动阈值
void RecodeAngleTask(void)
{
    // 使用 static 保持跨函数调用的状态
    static float last_sample_angle[motorTotalAngleSetCNT] = {0}; // 上一次采样角度(用于判断静止)
    static uint32_t idle_timer[motorTotalAngleSetCNT] = {0};     // 静止计时器
    static uint8_t is_first_run = 1;                             // 首次运行标志

    // 1. 首次运行初始化 (避免刚上电误判为剧烈运动)
    if (is_first_run) {
        for (uint8_t i = 0; i < idx; i++) {
            GenericMotorData_s *m = motor_recoder_data[i];
            if (m->type == DJI_MOTOR) last_sample_angle[i] = m->data.dji.dji_motor->measure.total_angle;
            else if (m->type == HT_MOTOR) last_sample_angle[i] = m->data.ht.ht_motor->measure.total_angle;
            // ... 其他电机类型
        }
        is_first_run = 0;
        return; // 第一次只初始化，不进行逻辑判断
    }

    // 2. 遍历所有电机
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


        // --- B. 静止检测逻辑 ---
        // 如果 (当前角度 - 上次采样) 很小，认为电机在静止状态
        if (fabsf(current_angle - last_sample_angle[i]) < IDLE_CHECK_DIFF) {
            idle_timer[i] += RECODE_PERIOD_MS;
        } else {
            // 电机在动，重置计时器
            idle_timer[i] = 0;
        }
        
        // 更新采样值供下次比较
        last_sample_angle[i] = current_angle;

        // --- C. 写入触发逻辑 ---
        // 只有当：(1)静止时间足够长 AND (2)与EEPROM存的值差异大
        if (idle_timer[i] >= IDLE_TIMEOUT_MS)
        {
            if (saved_angle_ptr != NULL && fabsf(current_angle - *saved_angle_ptr) > MIN_SAVE_DIFF)
            {
                // 执行写入 (内部包含 osDelay(5))
                AT24C02_WriteData(i * 4, (uint8_t *)&current_angle, sizeof(float));
                
                // 更新“上次保存值”，防止重复写入
                *saved_angle_ptr = current_angle; 
            }
        }
    }
}