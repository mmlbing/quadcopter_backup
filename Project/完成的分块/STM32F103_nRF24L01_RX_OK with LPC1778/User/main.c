#include "stm32f10x.h"
#include "sysinit.h"
#include "quad_uart.h"
#include "nRF24L01.h"
#include "delay.h"

int main(void)
{
	uint8_t tdata[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	SysClockInit();
	Initial_UART1(0);
	nRF24L01_SPI2_Init();
	while(nRF24L01_Check() == 1)
	{
		PrintChar("Check failed!");
		delay_ms(500);
	}
	PrintChar("Check OK!");
	nRF24L01_RX_Mode();	
	while(1)
	{
		nRF24L01_RxPacket(tdata);
		/*if(nRF24L01_RxPacket(tdata))
				Uart1_Put_Buf(tdata,4);
		else
			PrintChar("Receive none!");
		delay_ms(500);*/
	}
}
