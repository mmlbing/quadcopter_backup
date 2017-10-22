#include "stm32f10x.h"

#define	LED_ON		GPIOB->BSRR |= 0x20
#define	LED_OFF		GPIOB->BRR  |= 0x20

void LEDInit(void);
void LEDState(void);
