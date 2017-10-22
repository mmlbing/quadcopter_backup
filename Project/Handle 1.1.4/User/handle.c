#include "handle.h"

uint32_t IntNum = 0;

extern uint16_t ADCData[4];
char LockState = LOCKED;

uint16_t powerLast=0;
uint8_t TxData[11];
/**
  * @brief  发送控制数据.
  * @param  None
  * @retval None
  */
void TxConrtolData(void)
{
	dataPrepare();
	nRF24L01_TxPacket(TxData);
}

/**
  * @brief  发送锁命令.
  * @param  None
  * @retval None
  */
void TxConrtolCommandLock(void)
{
	if(LockState == LOCKED)
	{
		LockState = UNLOCKED;
		TxData[1] = 0x12;
	}
	else
	{
		LockState = LOCKED;
		TxData[1] = 0x11;
	}
	TxData[0] = 0xAA;
	nRF24L01_TxPacket(TxData);
}

/**
  * @brief  发送控制命令.
  * @param  None
  * @retval None
  */
void TxConrtolCommand(void)
{
	
}

static void dataPrepare(void)
{
	int16_t tmp;
	tmp = (4095 - ADCData[2])/6.4;		//转化为0--640的值
	TxData[1] = tmp>>8;
	TxData[2] = tmp&0xFF;
	
	tmp = (2058 - ADCData[0])/20;
	TxData[3] = tmp>>8;
	TxData[4] = tmp&0xFF;
	
	tmp = (ADCData[1] - 2036)/20;
	TxData[5] = tmp>>8;
	TxData[6] = tmp&0xFF;
	
	tmp = (ADCData[3] - 2048)/20;	//暂时没用
	TxData[7] = tmp>>8;
	TxData[8] = tmp&0xFF;
	
	TxData[0] = 0xA5;
}
