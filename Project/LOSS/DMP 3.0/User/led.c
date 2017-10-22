#include "led.h"

/**
  * @brief  ÉèÖÃLED.
	* @detail	
  * @param  None
  * @retval None
  */
void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
}

void LEDChangeState(void)
{
	if( (GPIOB->ODR)&0x20 )
		LED_OFF;
	else
		LED_ON;
}
