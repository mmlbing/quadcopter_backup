#include "stm32f10x.h"
#include "system_init.h"
#include "handleAD.h"
#include "nRF24L01.h"
#include "keys.h"
#include "delay.h"

int main()
{
	SysClockInit();
	SystickInit();
	ADC12_Init();
	nRF24L01_SPI1_Init();
	while( nRF24L01_Check() );
	nRF24L01_TX_Mode();
	
	Key4_IRQ_Init();
	SystickEnable();
	while(1)
	{
	}
}
