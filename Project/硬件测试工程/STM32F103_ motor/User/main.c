#include "stm32f10x.h"
#include "sysinit.h"

int main()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SysClockInit();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1);
	//QuadTimerInit();
	while(1)
	{
	}
}


/*void delay(uint16_t num)
{
	uint16_t i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<num;j++);
}*/
