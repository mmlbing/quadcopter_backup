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
	SysClockInit();
	//Initial_UART1(0);
	LEDInit();
	nRF24L01_SPI2_Init();
	while(nRF24L01_Check() == 1)
	{
		//PrintChar("Check failed!");
		delay_ms(500);
	}
	//PrintChar("Check OK!");
	LED_ON;
	nRF24L01_RX_Mode();	
	//nRF24L01_IRQ_Init();
	//nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS,0xFF);
	while(1)
	{
		if(RxNum == 255)
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
		lostNum = RxData[0] - RxNum;
		//delay_ms(10);
		//nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS,0xFF);
	}
}
