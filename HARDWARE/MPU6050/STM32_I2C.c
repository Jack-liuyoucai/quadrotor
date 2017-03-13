#include "STM32_I2C.h"

#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver	  1
#define SCL_H    (GPIOB->ODR |= (1<<4))
#define SCL_L    (GPIOB->ODR &=~(1<<4))

#define SDA_H    (GPIOB->ODR |= (1<<3))
#define SDA_L    (GPIOB->ODR &=~(1<<3))

#define SDA_read      (GPIOB->IDR &0X0008)

extern u8 delays;
void I2C_delay(void)
{
  //  volatile int i = 20;
    u8  d=0;
	d=delays;
    while (d)
        d--;
}

static bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return FALSE;
    SDA_L;
	I2C_delay();
    if (SDA_read)
        return FALSE;
    SDA_L;
    I2C_delay();
    return TRUE;
}

static void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

static void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

static bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) {
        SCL_L;
        return FALSE;
    }
    SCL_L;
    return TRUE;
}

static void I2C_SendByte(u8 byte)
{
    u8 i = 8;
    while (i--) {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

static u8 I2C_ReceiveByte(void)
{
    u8 i = 8;
    u8 byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

void i2cInit(void)
{
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟		   	 
	GPIOB->CRL&=0XFFF00FFF;
	GPIOB->CRL|=0X00066000; //PB3 4 开漏输出 2M    	
}

bool i2cWriteBuffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return FALSE;
        }
    }
    I2C_Stop();
    return TRUE;
}

bool i2cWrite(u8 addr, u8 reg, u8 data)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return TRUE;
}

bool i2cRead(u8 addr, u8 reg, u8 len, u8 *buf)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return TRUE;
}

void init_HMC5883() 
{
	I2C_Start();
	I2C_SendByte(0X3C);
	I2C_WaitAck();
	I2C_SendByte(0x02);
	I2C_WaitAck();
	I2C_SendByte(0x00);
	I2C_WaitAck();
	I2C_Stop();
}

void HMC5883_read(short *hmx,short *hmy,short *hmz)
{
	u8 buf[6]={0};
	u8 i=0;
	I2C_Start();                          //起始信号
	I2C_SendByte(0x3C);           //发送设备地址+写信号
	I2C_WaitAck();
	I2C_SendByte(0x03);                   //发送存储单元地址，从0x3开始	
	I2C_WaitAck();
	I2C_Start();                          //起始信号
	I2C_SendByte(0x3C+1);         //发送设备地址+读信号
	I2C_WaitAck();
	for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
	{		
	    buf[i] = I2C_ReceiveByte();          //BUF[0]存储数据
		if (i == 5)
		{
			I2C_NoAck();                //最后一个数据需要回NOACK
	    }
		else
		{
			I2C_Ack();                //回应ACK
		}
	}
	I2C_Stop();                          //停止信号
	*hmx=buf[0] << 8 | buf[1]; //Combine MSB and LSB of X Data output register
	*hmz=buf[2] << 8 | buf[3]; //Combine MSB and LSB of Z Data output register
	*hmy=buf[4] << 8 | buf[5]; //Combine MSB and LSB of Y Data output register
}
