#include "stm32f10x.h"

#define	LED_OFF		GPIOA->BSRR |= 0x04
#define	LED_ON		GPIOA->BRR  |= 0x04

void LEDInit(void);
void LEDChangeState(void);
