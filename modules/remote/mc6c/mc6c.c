#include "mc6c.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"


#define MC_FRAME_SIZE 25u // 遥控器接收的buffer大小

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 遥控器数据
static MC_ctrl_t mc_ctrl[2];
static uint8_t mc_init_flag = 0;

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *mc_usart_instance;
static DaemonInstance *mc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (abs(*(&mc_ctrl[TEMP].rocker_r_ + i)) > MC_DATA_MAX)
            *(&mc_ctrl[TEMP].rocker_r_ + i) = 0;
    }

}

static uint8_t mc_rx_sta=0;
/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_mc(const uint8_t *sbus_buf)
{
    if ((sbus_buf[0] == 0x0F)&&sbus_buf[24]==0x00) mc_rx_sta = 1;
    else mc_rx_sta=0;

    if(mc_rx_sta==1)
    {
        mc_ctrl[TEMP].rocker_r_= ((sbus_buf[1] | sbus_buf[2] << 8) & 0x07FF) - MC_DATA_MID;							//右拨杆水平
        mc_ctrl[TEMP].rocker_l1= ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) - MC_DATA_MID;					//右拨杆垂直
        mc_ctrl[TEMP].rocker_r1= ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - MC_DATA_MID;	//左拨杆垂直
        mc_ctrl[TEMP].rocker_l_= ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - MC_DATA_MID;					//左拨杆水平

        mc_ctrl[TEMP].switch_l= ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF);
        mc_ctrl[TEMP].switch_r= ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF);

        mc_ctrl[TEMP].none[0] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
        mc_ctrl[TEMP].none[1] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
        // mc_ctrl[TEMP].none[2] = ((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF);
        // mc_ctrl[TEMP].none[3] = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF);
        // mc_ctrl[TEMP].none[4] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2 | sbus_buf[16] << 10) & 0x07FF);
        // mc_ctrl[TEMP].none[5] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7) & 0x07FF);
        // mc_ctrl[TEMP].none[6] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4) & 0x07FF);
        // mc_ctrl[TEMP].none[7] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1 | sbus_buf[20] << 9) & 0x07FF);
        // mc_ctrl[TEMP].none[8] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6) & 0x07FF);
        // mc_ctrl[TEMP].none[9] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3) & 0x07FF);
        
        if (abs(mc_ctrl[TEMP].rocker_l_) <= 50) mc_ctrl[TEMP].rocker_l_ = 0;
        if (abs(mc_ctrl[TEMP].rocker_l1) <= 50) mc_ctrl[TEMP].rocker_l1 = 0;
        if (abs(mc_ctrl[TEMP].rocker_r_) <= 50) mc_ctrl[TEMP].rocker_r_ = 0;
        if (abs(mc_ctrl[TEMP].rocker_r1) <= 50) mc_ctrl[TEMP].rocker_r1 = 0;

        // 断连/脏帧时该通道会偶发固定异常值，直接在源头滤掉
        if (mc_ctrl[TEMP].rocker_r1 == -960) mc_ctrl[TEMP].rocker_r1 = 0;

        mc_data_change(mc_ctrl[TEMP].switch_l);
        mc_data_change(mc_ctrl[TEMP].switch_r);
        
        RectifyRCjoystick();
        mc_rx_sta = 0;                        // 准备下一次接收
    }
    memcpy(&mc_ctrl[LAST], &mc_ctrl[TEMP], sizeof(MC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void MCRxCallback()
{
    DaemonReload(mc_daemon_instance);         // 先喂狗
    sbus_to_mc(mc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void MCLostCallback(void *id)
{
    memset(mc_ctrl, 0, sizeof(mc_ctrl)); // 清空遥控器数据
    USARTServiceInit(mc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

MC_ctrl_t *MCControlInit(UART_HandleTypeDef *mc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = MCRxCallback;
    conf.usart_handle = mc_usart_handle;
    conf.recv_buff_size = MC_FRAME_SIZE;
    mc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = MCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    mc_daemon_instance = DaemonRegister(&daemon_conf);

    mc_init_flag = 1;
    return mc_ctrl;
}

uint8_t MCControlIsOnline()
{
    if (mc_init_flag)
        return DaemonIsOnline(mc_daemon_instance);
    return 0;
}