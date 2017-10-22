#include "stm32f10x.h"
#include "nRF24L01.h"
#include "delay.h"

uint8_t Tx_Buf[6] = {0x1A, 0x2A, 0x3A, 0x4A, 0x5A, 0x6A};

void mdelay_ms(uint16_t j);
int main()
{
	nRF24L01_SPI1_Init();
	while( nRF24L01_Check() );
	nRF24L01_TX_Mode();
	
	while(1)
	{
		nRF24L01_TxPacket(Tx_Buf);
		mdelay_ms(300);
	}
}


void mdelay_ms(uint16_t j)
{
	uint32_t i;
	for(;j>0;j--)
		for(i=28571;i>0;i--);
}
