#ifndef AT24C02
#define AT24C02

#include "main.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "HT04.h"
#include "jz_motor.h"
#include "LK9025.h"
#include "cmsis_os.h"  // <--- 【必须添加】解决 implicit declaration of function 'osDelay'
#include <math.h>      // <--- 【建议添加】你用了 fabsf
#include <stdlib.h>    // <--- 【建议添加】你用了 malloc
#include <stdint.h>

#define motorTotalAngleSetCNT 20

#define AT24C02_WADDR 0xA0  // AT24C02 写地址
#define AT24C02_RADDR 0xA1  // AT24C02 读地址

typedef enum
{
    DJI_MOTOR = 0,
    DM_MOTOR,
    HT_MOTOR,
    JZ_MOTOR,
    LK_MOTOR
} MotorType_e;

typedef struct
{
    DJIMotorInstance *dji_motor;
    float angle;
} DJIMotorAngleData_s;

typedef struct
{
    DMMotorInstance *dm_motor;
    float angle;
} DmMotorAngleData_s;
typedef struct
{
    HTMotorInstance *ht_motor;
    float angle;
} HTMotorAngleData_s;
typedef struct
{
    JZMotorInstance *jz_motor;
    float angle;
} JZMotorAngleData_s;
typedef struct
{
    LKMotorInstance *lk_motor;
    float angle;
} LKMotorAngleData_s;
typedef struct {
    MotorType_e type; // 必须有类型标志
    union {
        DJIMotorAngleData_s dji;
        DmMotorAngleData_s  dm;
        HTMotorAngleData_s  ht;
        JZMotorAngleData_s  jz;
        LKMotorAngleData_s  lk;
        // ... 其他结构体
    } data;
} GenericMotorData_s;

typedef struct
{
    MotorType_e type; // 必须有类型标志
    union {
        DJIMotorInstance *dji;
        DMMotorInstance *dm;
        HTMotorInstance *ht;
        JZMotorInstance *jz;
        LKMotorInstance *lk;
        // ... 其他结构体
    } data;
} Motor_Recoder_Init_Config_s;

void readAllMotorAngle();

// void writeDjiMotorAngle(uint8_t idx,int32_t angle);

void setAllMotorZero();
void motorDataInit();
void motorRecoderRegister(Motor_Recoder_Init_Config_s *config);

void RecodeAngleTask();
#endif // AT24C02
