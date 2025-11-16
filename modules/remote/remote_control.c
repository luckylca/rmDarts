#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小
// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

int16_t rc_cyy[6]; // 遥控器数据   
// 拉力传感器数据
double F_data[2] = {0};
static uint8_t F_init_flag = 0; 
static int32_t weight1=0;
static double  result1=0;
static int32_t weight2=0;
static double  result2=0;
double temp=0;
double adjusted_weight=0;
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

// 拉力传感器用的串口实例
static USARTInstance *F_usart_instance;
static DaemonInstance *F_daemon_instance;
extern uint8_t rs485buf[5];
/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{


    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}


int16_t remap_left_and_right(int16_t value)
{
    if(value >0)
    {
        return 2;
    }
    else if(value <0)
    {
        return 1;
    }
    else
    {
        return 3;
    }
}

int16_t remap_channel_8(int16_t value)
{
    if(value >=0)
    {
        return 1;
    }
    else if(value <0)
    {
        return 0;
    }
  
}


int16_t dead_line(int16_t value)
{
    if(abs(value)<10)
    {
        return 0;
    }
    else
    {
        return value;
    }
}

// sbus解析
static void sbus_to_rc_cyy(const uint8_t *sbus_buf)
{

    for(int i=0;i<25;i++)
    {   
        if(sbus_buf[i] == 0x0f)
        {
            // 改回源代码需要改为uint16_t
            rc_ctrl[TEMP].rc.rocker_r_ = dead_line((int)((((sbus_buf[i+1] | sbus_buf[i+2] << 8) & 0x07FF) - RC_CH_VALUE_MIDDLE_CYY) *660/800 ));                              //!< Channel 0
            rc_ctrl[TEMP].rc.rocker_r1 = dead_line((int)((((sbus_buf[i+3] >> 6 | sbus_buf[i+4] << 2 | sbus_buf[5] << 10) & 0x07FF) - RC_CH_VALUE_MIDDLE_CYY) *660/800)); //!< Channel 1
            rc_ctrl[TEMP].rc.rocker_l_ = dead_line((int)((((sbus_buf[i+2] >> 3 | sbus_buf[i+3] << 5) & 0x07FF)- RC_CH_VALUE_MIDDLE_CYY)*660/800));
            rc_ctrl[TEMP].rc.rocker_l1 = dead_line((int)((((sbus_buf[i+5] >> 1 | sbus_buf[i+6] << 7) & 0x07FF) - RC_CH_VALUE_MIDDLE_CYY)*660/800));
            rc_ctrl[TEMP].rc.dial      = dead_line((int)((((sbus_buf[i+9] >> 2 | sbus_buf[i+10] << 6) & 0x07FF) - RC_CH_VALUE_MIDDLE_CYY)*660/800)); // 左侧拨轮                           
            RectifyRCjoystick();
            // 开关,0左1右
            rc_ctrl[TEMP].rc.switch_right = remap_left_and_right((int)((((sbus_buf[i+7] >> 7 | sbus_buf[i+8] << 1 | sbus_buf[9] << 9) & 0x07FF) - RC_CH_VALUE_MIDDLE_CYY)*660/800));     //!< Switch right
            rc_ctrl[TEMP].rc.switch_left =  remap_left_and_right((int)((((sbus_buf[i+6] >> 4 | sbus_buf[i+7] << 4) & 0x07FF)    - RC_CH_VALUE_MIDDLE_CYY)*660/800)); //!< Switch left
            rc_ctrl[TEMP].mouse.x = remap_channel_8((int)(( ((sbus_buf[i+10] >> 5 | sbus_buf[i+11] << 3) & 0x07FF)  - RC_CH_VALUE_MIDDLE_CYY)*660/800));
            if(rc_ctrl[TEMP].rc.dial == -660)
            {
                rc_ctrl[TEMP].rc.switch_right = 2;
                rc_ctrl[TEMP].rc.switch_left = 2;
            }
            memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

            break;
        }
    }
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗

    if(USE_CYY_SBUS_REMOTE)
    {
        sbus_to_rc_cyy(rc_usart_instance->recv_buff); // 使用Sbus进行协议解析
    } 
    else
    {
        sbus_to_rc(rc_usart_instance->recv_buff); // 使用DBUS进行协议解析
    }
        
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = (void *)rc_usart_handle, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}



////////////////////////////////////////以下为拉力传感器（仿照遥控////////////////////////////////////////////////////
/**
 * @brief 拉力传感器数据解析
 *
 * @param F_data_buf 接收buffer
 */

int32_t decode_weight(uint8_t X1, uint8_t X2, uint8_t X3, uint8_t X4, uint8_t X5) 
{
    // 将X1～X5的高4位去掉，低4位保留实际重量数据
    int32_t weight = (X5 - 0x30) * 65536 +
                     (X4 - 0x30) * 4096 +
                     (X3 - 0x30) * 256 +
                     (X2 - 0x30) * 16 +
                     (X1 - 0x30);
    return weight;
}

// 解析X6的状态信息并根据小数点位置调整重量显示，返回调整后的重量数值
double decode_status(uint8_t X6, int32_t weight) 
{
    // Bit7-Bit6 固定为 01，不需要处理
    // Bit5: 超载符号
    uint8_t overload = (X6 >> 5) & 0x01;
    // Bit4: 零点标志
    uint8_t zero_flag = (X6 >> 4) & 0x01;
    // Bit3: 稳定标志
    uint8_t stable_flag = (X6 >> 3) & 0x01;
    // Bit2: 重量符号
    uint8_t weight_sign = (X6 >> 2) & 0x01;
    // Bit1-Bit0: 小数点位置
    uint8_t decimal_point = X6 & 0x03;

    // 如果是负数，根据符号位调整重量符号
    if (weight_sign) {
        weight = -weight;
    }

    // 根据小数点位置调整重量显示
    adjusted_weight = weight;
    switch (decimal_point) {
        case 0:  // 无小数点
            return (double)weight;
        case 1:  // 1 位小数
            adjusted_weight = weight / 10.0;
            return adjusted_weight;
        case 2:  // 2 位小数
            adjusted_weight = weight / 100.0;
            return adjusted_weight;
        case 3:  // 3 位小数
            adjusted_weight = weight / 1000.0;
            return adjusted_weight;
        default:
            return (double)weight;  // 默认返回原始重量
    }
}



static void f_data_solve(const uint8_t *F_data_buf)
{
    weight1 = decode_weight(F_data_buf[2],F_data_buf[3],F_data_buf[4],F_data_buf[5],F_data_buf[6]);
    result1 = decode_status(F_data_buf[7], weight1); 
    F_data[0] = result1;
    
    weight2 = decode_weight(F_data_buf[10],F_data_buf[11],F_data_buf[12],F_data_buf[13],F_data_buf[14]);
    result2 = decode_status(F_data_buf[15], weight2);
    F_data[1] = result2;
}

/**
 * @brief 对F_DATA的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void F_RxCallback()
{
    DaemonReload(F_daemon_instance);         // 先喂狗
    f_data_solve(F_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 拉力传感器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void FLostCallback(void *id)
{
    F_data[0] = 0; // 清空拉力传感器数据
    F_data[1] = 0;
    HAL_UART_Transmit(&huart6, rs485buf, 5, 1000);
    USARTServiceInit(F_usart_instance); // 尝试重新启动接收
    LOGWARNING("[F] remote control lost");
}

double* F_Init(UART_HandleTypeDef *F_usart_handle)
{
    USART_Init_Config_s conf_F;
    conf_F.module_callback = F_RxCallback;
    conf_F.usart_handle = F_usart_handle;
    conf_F.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    F_usart_instance = USARTRegister(&conf_F);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s F_daemon_conf = {
        .reload_count = 200, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = FLostCallback,
        .owner_id = (void *)F_usart_handle, // 只有1个遥控器,不需要owner_id
    };
    F_daemon_instance = DaemonRegister(&F_daemon_conf);

    F_init_flag = 1;
    return F_data;
}

uint8_t F_IsOnline()
{
    if (F_init_flag)
        return DaemonIsOnline(F_daemon_instance);
    return 0;
}