#include "stm32f10x.h"
#include "sysinit.h"
#include "quad_uart.h"
#include "nRF24L01.h"
#include "delay.h"
#include "led.h"

int main(void)
{
	uint8_t tdata[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
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
	while(1)
	{
		tdata[1] = 0x00;
		nRF24L01_RxPacket(tdata);
		if(tdata[1] == 0x2A)
			LEDChangeState();
		delay_ms(200);
	}
}
