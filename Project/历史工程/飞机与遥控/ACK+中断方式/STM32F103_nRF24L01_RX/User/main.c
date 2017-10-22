#include "stm32f10x.h"
#include "sysinit.h"
#include "quad_uart.h"
#include "nRF24L01.h"
#include "delay.h"
#include "led.h"

extern uint8_t RxData[4];
uint32_t unRxNum = 0, lostNum = 0;
uint8_t RxNum = 0;
int main(void)
{
	//uint8_t state;
	SysClockInit();
	LEDInit();
	nRF24L01_SPI2_Init();
	while(nRF24L01_Check() == 1)
	{
		delay_ms(500);
	}
	
	LED_ON;
	nRF24L01_RX_Mode();
	
	//清除终端标志和FIFO
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS, 0xFF);
	nRF24L01_Write_Reg(FLUSH_RX,0xFF);
	while(1)
	{
		/*if(RxNum == 255)
		{
			RxNum = 0;
		}
		if (nRF24L01_RxPacket(RxData))
		{
			unRxNum++;
		}
		else
		{
			RxNum++;
			LEDChangeState();
		}
		lostNum = RxData[0] - RxNum;*/
		//delay_ms(10);
		//nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS,0xFF);
	}
}
