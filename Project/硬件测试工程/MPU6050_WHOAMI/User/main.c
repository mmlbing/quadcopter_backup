#include "stm32f10x.h"
#include "STM32_I2C.h"
#include "quad_uart.h"
#include "MPU6050.h"

void delay(uint16_t num)
{
	uint16_t i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<num;j++);
}

int main()
{
	//float PRY[3];
	uint8_t who;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_All);
	
	//Initial_UART1(57600);
	I2CInit();
	while(1)
	{
		I2CReadBuffer(0x69,0x75,1,&who);
		if( who==0x68 )
		{
			while(1)
			{
				GPIO_SetBits(GPIOB, GPIO_Pin_5);
				delay(1000);
				GPIO_ResetBits(GPIOB, GPIO_Pin_5);
				delay(1000);
			}
		}
	}
}


