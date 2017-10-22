#include "stm32f10x.h"
#include "sysinit.h"
#include "quad_uart.h"
#include "nRF24L01.h"
#include "delay.h"

int main()
{
	//uint8_t dat = 0x55,state = 0;
	SysClockInit();
	Initial_UART1(0);
	nRF24L01_SPI2_Init();
	
	while(1)
	{
		if( nRF24L01_Check() == 0)
			UART1_Put_Char(0xAA);
		UART1_Put_Char(0x55);
		delay_ms(1000);
	}
}
