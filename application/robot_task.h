/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "dmmotor.h"
#include "buzzer.h"
#include "remote_control.h"
#include "bsp_log.h"
#include "robot_cmd.h" 
#include "encoder.h"
#include "at24c02.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId encoderTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId uart1TaskHandle;
osThreadId uart6TaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartENCODERTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartRecodeAngle(void const *argument);
void StartUART1TASK(void const *argument);
void StartUART6TASK(void const *argument);



/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(encodertask, StartENCODERTASK, osPriorityNormal, 0, 128);
    encoderTaskHandle = osThreadCreate(osThread(encodertask), NULL);

    osThreadDef(recodeangletask, StartRecodeAngle, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(recodeangletask), NULL);
    // 因为要用串口六测试，先把ui禁止了
    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    // osThreadDef(uart1task, StartUART1TASK, osPriorityNormal, 0, 256);
    // uart1TaskHandle = osThreadCreate(osThread(uart1task), NULL);

    // osThreadDef(uart6task, StartUART6TASK, osPriorityNormal, 0, 256);
    // uart6TaskHandle = osThreadCreate(osThread(uart6task), NULL);


    DMMotorControlInit();//为所有的达妙电机注册任务
    // HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
        VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartENCODERTASK(void const *argument)
{
    static float encoder_dt;
    static float encoder_start;
    LOGINFO("[freeRTOS] ENCODER Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        encoder_start = DWT_GetTimeline_ms();
        EncoderTask();
        encoder_dt = DWT_GetTimeline_ms() - encoder_start;
        if (encoder_dt > 5)
            LOGERROR("[freeRTOS] ENCODER Task is being DELAY! dt = [%f]", &encoder_dt);
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartRecodeAngle(void const *argument)
{
    static float recoder_dt;
    static float recoder_start;
    LOGINFO("[freeRTOS] ENCODER Task Start");
    motorDataInit();
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        recoder_start = DWT_GetTimeline_ms();
        RecodeAngleTask();
        recoder_dt = DWT_GetTimeline_ms() - recoder_start;
        if (recoder_dt > 15)
            LOGERROR("[freeRTOS] ENCODER Task is being DELAY! dt = [%f]", &recoder_dt);
        osDelay(100);
    }
}

__attribute__((noreturn)) void StartUART1TASK(void const *argument)
{
    LOGINFO("[freeRTOS] UART1 Task Start");
    for (;;)
    {
        // 读取串口一的数据
        // uint8_t data;
        // if (HAL_UART_Receive(&huart1, &data, 1, HAL_MAX_DELAY) == HAL_OK)
        // {
        //     // 处理接收到的数据
        //     LOGINFO("Received data: %d", data);
        // }
        // uart1Task();
           // 遥控器数据,初始化时返回
        osDelay(1);
    }
} 

__attribute__((noreturn)) void StartUART6TASK(void const *argument)
{
    LOGINFO("[freeRTOS] UART6 Task Start");
    for (;;)
    {

        uart6Task();
           // 遥控器数据,初始化时返回
        osDelay(1);
    }
} 
