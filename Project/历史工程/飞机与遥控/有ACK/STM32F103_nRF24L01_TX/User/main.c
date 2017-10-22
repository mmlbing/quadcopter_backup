#include "stm32f10x.h"
#include "nRF24L01.h"
#include "delay.h"

uint8_t Tx_Buf[4] = {0x00, 0x01, 0x01, 0x01};

void mdelay_ms(uint16_t j);
int main()
{
	nRF24L01_SPI1_Init();
	while( nRF24L01_Check() );
	nRF24L01_TX_Mode();
	
	while(1)
	{
		if(Tx_Buf[0] == 255)
			Tx_Buf[0] = 0;
		nRF24L01_TxPacket(Tx_Buf);
		Tx_Buf[0]++;
		mdelay_ms(500);
	}
}


void mdelay_ms(uint16_t j)
{
	uint32_t i;
	for(;j>0;j--)
		for(i=14231;i>0;i--);
}
