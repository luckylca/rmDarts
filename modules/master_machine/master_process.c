/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static USARTInstance *vision_usart_instance;

// 添加字节处理的缓冲区和计数器
static uint8_t vision_recv_buffer[sizeof(Vision_Recv_s)];  // 用于接收数据的缓冲区
static uint16_t vision_recv_index = 0;                     // 当前处理位置

#define ERR_FILTER_LEN 20
static float err_filter_buf[ERR_FILTER_LEN] = {0};
static uint8_t filter_idx = 0;

/**
 * @brief 视觉误差滑动均值滤波
 * @param new_value 新的误差值
 * @return float 滤波后的误差值
 */
static float VisionErrFilter(float new_value)
{
    // 更新缓存
    err_filter_buf[filter_idx] = new_value;
    filter_idx++;
    
    if (filter_idx >= ERR_FILTER_LEN)
    {
        filter_idx = 0;
    }

    // 计算平均值
    float sum = 0.0f;
    for (uint8_t i = 0; i < ERR_FILTER_LEN; i++)
    {
        sum += err_filter_buf[i];
    }
    
    return sum / (float)ERR_FILTER_LEN;
}

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

/**
 * @brief 处理接收到的数据，按字节分析
 */
static void ProcessReceivedData()
{
    // 喂狗，表示通信正常
    DaemonReload(vision_daemon_instance);
    
    // 获取原始数据，逐字节处理
    uint8_t *raw_data = vision_usart_instance->recv_buff;
    
    // 处理接收到的每个字节
    for (uint16_t i = 0; i < sizeof(Vision_Recv_s); i++)
    {
        // 使用单字节处理函数存储数据
        if (process_single_byte(raw_data[i], vision_recv_buffer, &vision_recv_index, sizeof(Vision_Recv_s)))
        {
            // 缓冲区已满，数据包完整，复制数据
            memcpy(&recv_data, vision_recv_buffer, sizeof(Vision_Recv_s));
            
            // 对err_of_pix进行滑动均值滤波
            recv_data.err_of_pix = VisionErrFilter(recv_data.err_of_pix);

            // 设置标志位
            recv_data.target_state = TARGET_CONVERGING;
            
            // vision_recv_index 已在 process_single_byte 函数中重置
        }
    }
}

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 */
static void DecodeVision()
{
    // 处理接收到的数据
    ProcessReceivedData();
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = sizeof(Vision_Recv_s);  // 接收缓冲区大小
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 初始化接收缓冲区和索引
    memset(vision_recv_buffer, 0, sizeof(Vision_Recv_s));
    vision_recv_index = 0;

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // 初始化接收数据
    memset(&recv_data, 0, sizeof(Vision_Recv_s));
    recv_data.target_state = NO_TARGET;

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
    // 析构后的陷阱需要特别注意!
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    // TODO: code to set flag_register
    flag_register = 30 << 8 | 0b00000001;
    // 将数据转化为seasky协议的数据包
    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;
// 添加VCP处理的缓冲区和计数器
static uint8_t vcp_buffer[sizeof(Vision_Recv_s)];
static uint16_t vcp_buffer_index = 0;

int16_t cyy_test_data = 0;
/**
 * @brief 处理接收到的VCP数据
 * @param recv_len 接收到的数据长度
 */
static void DecodeVision(uint16_t recv_len) {
    DaemonReload(vision_daemon_instance);
    
    static uint8_t state = 0; 
    static uint8_t count = 0;
    static uint8_t calc_checksum = 0;
    const uint8_t DATA_LEN = sizeof(Vision_Recv_s);

    for (uint16_t i = 0; i < recv_len; i++) {
        uint8_t byte = vis_recv_buff[i];
        
        switch (state) {
            case 0: // 找固定帧头 0xAA
                if (byte == 0xAA) {
                    state = 1;
                    count = 0; // 准备开始计数
                    calc_checksum = 0; // 重置校验和
                }
                break;
            
            case 1: // 接收数据体 (16字节)
                vcp_buffer[count++] = byte;
                calc_checksum += byte;
                if (count >= DATA_LEN) {
                    state = 2; // 收够了，去检查校验位
                }
                break;
                
            case 2: // 接收校验位 (假设协议里有校验位)
                if(byte == calc_checksum) {
                    state = 3; // 校验通过，去检查帧尾
                } else {
                    // 校验失败，重置状态机
                    state = 0;
                }
                break;

            case 3: // 检查帧尾 0x55
                if (byte == 0x55) {
                    // 只有走到这里，且帧尾正确，才认为数据有效
                    memcpy(&recv_data, vcp_buffer, DATA_LEN);
                    // 对err_of_pix进行滑动均值滤波
                    recv_data.err_of_pix = VisionErrFilter(recv_data.err_of_pix);
                }
                state = 0; 
                break;
            default:
                state = 0; 
                break;
        }
    }
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 初始化接收缓冲区和索引
    memset(vcp_buffer, 0, sizeof(Vision_Recv_s));
    vcp_buffer_index = 0;

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    // 初始化接收数据
    memset(&recv_data, 0, sizeof(Vision_Recv_s));


    return &recv_data;
}

void VisionSend()
{
    // 直接发送send_data结构体数据
    USBTransmit((uint8_t *)&send_data, sizeof(Vision_Send_s));
}

#endif // VISION_USE_VCP
