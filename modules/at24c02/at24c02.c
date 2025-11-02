#include "at24c02.h"

int32_t DjiMotorTotalAngleSet[motorTotalAngleSetCNT] = {0};
int32_t DmMotorTotalAngleSet[motorTotalAngleSetCNT] = {0};

// 延时函数，模拟 I2C 时序
static void IIC_Delay(void)
{
    for (volatile int i = 0; i < 50; i++)
        ;
}

// 生成 I2C 起始信号
static void IIC_Start(void)
{
    IIC_SDA_HIGH();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SDA_LOW();
    IIC_Delay();
    IIC_SCL_LOW();
}

// 生成 I2C 停止信号
static void IIC_Stop(void)
{
    IIC_SDA_LOW();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SDA_HIGH();
    IIC_Delay();
}

// 等待应答信号
static uint8_t IIC_WaitAck(void)
{
    uint8_t ack;
    IIC_SDA_HIGH();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    ack = IIC_SDA_READ();
    IIC_SCL_LOW();
    return ack == GPIO_PIN_RESET ? 0 : 1; // 返回 0 表示应答成功
}

// 发送应答信号
static void IIC_Ack(void)
{
    IIC_SDA_LOW();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SCL_LOW();
}

// 发送非应答信号
static void IIC_NAck(void)
{
    IIC_SDA_HIGH();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SCL_LOW();
}

// 发送一个字节
void IIC_SendByte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        if (byte & 0x80)
            IIC_SDA_HIGH();
        else
            IIC_SDA_LOW();
        IIC_Delay();
        IIC_SCL_HIGH();
        IIC_Delay();
        IIC_SCL_LOW();
        byte <<= 1;
    }
}

// 接收一个字节
uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    IIC_SDA_HIGH(); // 释放 SDA
    for (int i = 0; i < 8; i++)
    {
        IIC_Delay();
        IIC_SCL_HIGH();
        IIC_Delay();
        byte = (byte << 1) | (IIC_SDA_READ() ? 1 : 0);
        IIC_SCL_LOW();
    }
    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return byte;
}

// AT24C02 写入一个字节
void AT24C02_WriteByte(uint16_t addr, uint8_t data)
{
    IIC_Start();
    IIC_SendByte(AT24C02_WADDR); // 发送写地址
    IIC_WaitAck();
    IIC_SendByte((uint8_t)addr); // 发送存储地址
    IIC_WaitAck();
    IIC_SendByte(data); // 发送数据
    IIC_WaitAck();
    IIC_Stop();
}

// AT24C02 读取一个字节
uint8_t AT24C02_ReadByte(uint16_t addr)
{
    uint8_t data;
    IIC_Start();
    IIC_SendByte(AT24C02_WADDR); // 发送写地址
    IIC_WaitAck();
    IIC_SendByte((uint8_t)addr); // 发送存储地址
    IIC_WaitAck();
    IIC_Start();
    IIC_SendByte(AT24C02_RADDR); // 发送读地址
    IIC_WaitAck();
    data = IIC_ReadByte(0); // 读取数据，发送非应答信号
    IIC_Stop();
    return data;
}

// AT24C02 写入多个字节
void AT24C02_Write(uint16_t addr, uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        AT24C02_WriteByte(addr + i, buffer[i]);
    }
}

// AT24C02 读取多个字节
void AT24C02_Read(uint16_t addr, uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        buffer[i] = AT24C02_ReadByte(addr + i);
    }
}

// 从 AT24C02 读取所有电机总角度数据
// DJI 电机: 地址 0x00-0x27 (10 个 int32_t)
// DM 电机:  地址 0x28-0x4F (10 个 int32_t)
void readAllMotorTotalAngleSet(void)
{
    // 读取 DJI 电机数据
    AT24C02_Read(0, (uint8_t *)DjiMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
    
    // 读取 DM 电机数据
    AT24C02_Read(motorTotalAngleSetCNT * sizeof(int32_t), (uint8_t *)DmMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
}

void writeAllMotorTotalAngleSet(void)
{
    // 写入 DJI 电机数据
    AT24C02_Write(0, (uint8_t *)DjiMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
    
    // 写入 DM 电机数据
    AT24C02_Write(motorTotalAngleSetCNT * sizeof(int32_t), (uint8_t *)DmMotorTotalAngleSet, motorTotalAngleSetCNT * sizeof(int32_t));
}

// 读取 DJI 电机总角度集合中的单个值
// idx: 索引 (0-9)
// 地址范围: 0x00-0x27
int32_t readDjiMotorTotalAngleSetByIndex(uint8_t idx)
{
    if (idx >= motorTotalAngleSetCNT)
        return 0; // 索引超出范围，返回 0

    int32_t value;
    uint16_t addr = idx * sizeof(int32_t);
    AT24C02_Read(addr, (uint8_t *)&value, sizeof(int32_t));
    return value;
}

// 写入 DJI 电机总角度集合中的单个值
// idx: 索引 (0-9)
// value: 要写入的值
// 地址范围: 0x00-0x27
void writeDjiMotorTotalAngleSetByIndex(uint8_t idx, int32_t value)
{
    if (idx >= motorTotalAngleSetCNT)
        return; // 索引超出范围，不执行操作

    uint16_t addr = idx * sizeof(int32_t);
    AT24C02_Write(addr, (uint8_t *)&value, sizeof(int32_t));
}

// 读取 DM 电机总角度集合中的单个值
// idx: 索引 (0-9)
// 地址范围: 0x28-0x4F
int32_t readDmMotorTotalAngleSetByIndex(uint8_t idx)
{
    if (idx >= motorTotalAngleSetCNT)
        return 0; // 索引超出范围，返回 0

    int32_t value;
    uint16_t addr = (motorTotalAngleSetCNT + idx) * sizeof(int32_t);
    AT24C02_Read(addr, (uint8_t *)&value, sizeof(int32_t));
    return value;
}

// 写入 DM 电机总角度集合中的单个值
// idx: 索引 (0-9)
// value: 要写入的值
// 地址范围: 0x28-0x4F
void writeDmMotorTotalAngleSetByIndex(uint8_t idx, int32_t value)
{
    if (idx >= motorTotalAngleSetCNT)
        return; // 索引超出范围，不执行操作

    uint16_t addr = (motorTotalAngleSetCNT + idx) * sizeof(int32_t);
    AT24C02_Write(addr, (uint8_t *)&value, sizeof(int32_t));
}