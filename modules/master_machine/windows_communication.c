// #include <windows.h>
#include <stdio.h>
#include "seasky_protocol.h"
#include "master_process.h"

HANDLE hSerial;
DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};

/**
 * @brief 初始化串口
 * 
 * @param port_name 串口名称，例如 "COM3"
 * @return int 成功返回0，失败返回-1
 */
int init_serial(const char *port_name) {
    hSerial = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Error opening serial port\n");
        return -1;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Error getting serial port state\n");
        CloseHandle(hSerial);
        return -1;
    }

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Error setting serial port state\n");
        CloseHandle(hSerial);
        return -1;
    }

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        printf("Error setting serial port timeouts\n");
        CloseHandle(hSerial);
        return -1;
    }

    return 0;
}

/**
 * @brief 发送数据到串口
 * 
 * @param data 待发送的数据
 * @param length 数据长度
 * @return int 成功返回写入的字节数，失败返回-1
 */
int send_data(uint8_t *data, uint16_t length) {
    DWORD bytes_written;
    if (!WriteFile(hSerial, data, length, &bytes_written, NULL)) {
        printf("Error writing to serial port\n");
        return -1;
    }
    return bytes_written;
}

/**
 * @brief 从串口接收数据
 * 
 * @param buffer 接收缓冲区
 * @param length 缓冲区长度
 * @return int 成功返回读取的字节数，失败返回-1
 */
int receive_data(uint8_t *buffer, uint16_t length) {
    DWORD bytes_read;
    if (!ReadFile(hSerial, buffer, length, &bytes_read, NULL)) {
        printf("Error reading from serial port\n");
        return -1;
    }
    return bytes_read;
}

/**
 * @brief 关闭串口
 */
void close_serial() {
    CloseHandle(hSerial);
}

/**
 * @brief 主函数，初始化串口，发送和接收数据
 * 
 * @return int 成功返回0，失败返回-1
 */
int main() {
    if (init_serial("COM3") != 0) {
        return -1;
    }

    uint8_t send_buff[VISION_SEND_SIZE];
    uint16_t tx_len;
    uint16_t flag_register = 30 << 8 | 0b00000001;
    Vision_Send_s send_data = {0}; // 初始化发送数据

    // 设置发送数据
    send_data.yaw = 1.0f;
    send_data.pitch = 2.0f;
    send_data.roll = 3.0f;

    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
    send_data(send_buff, tx_len);

    uint8_t recv_buff[VISION_RECV_SIZE];
    int bytes_received = receive_data(recv_buff, VISION_RECV_SIZE);
    if (bytes_received > 0) {
        uint16_t flags_register;
        Vision_Recv_s recv_data;
        get_protocol_info(recv_buff, &flags_register, (uint8_t *)&recv_data.pitch);
        printf("Received data: yaw=%f, pitch=%f, roll=%f\n", recv_data.yaw, recv_data.pitch, recv_data.roll);
    }

    close_serial();
    return 0;
}
