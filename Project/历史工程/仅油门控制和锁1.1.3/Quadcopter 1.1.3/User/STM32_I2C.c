#include "STM32_I2C.h"

static void I2C_delay(void)
{
    __nop();__nop();__nop();__nop();
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

static void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
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

static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

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

/***********************************************************************************************/
/*对外接口*/
/***********************************************************************************************/
void I2CInit(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(I2CPinGroup, ENABLE);
	gpio.GPIO_Pin = I2CPin;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(I2CGPIO, &gpio);
}

bool I2CWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Tx);
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


bool I2CWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    int i;
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Tx);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(buf[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return FALSE;
        }
    }
    I2C_Stop();
    return TRUE;
}

bool I2CReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return FALSE;
    I2C_SendByte(addr << 1 | I2C_Direction_Tx);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return FALSE;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Rx);
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
/*********************************/
/********MPU6050针对函数**********/
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	if(I2CWriteBuffer(addr,reg,len,data))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	if(I2CReadBuffer(addr,reg,len,data))
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
