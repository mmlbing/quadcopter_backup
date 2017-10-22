#include "stm32f10x.h"
#include "sysinit.h"
#include "quad_timer.h"
#include "led.h"
#include "delay.h"

int main()
{
	uint16_t tmp=0;
	SysClockInit();
	
	LEDInit();
	QuadTimerInit();
	//TIM_Cmd(TIM3, ENABLE);
	//TIM3->CCR1 = 2500;
	while(1)
	{
		/*if(tmp>5000)
		{
			tmp = 0;
		}
		LED_ON;
		TIM3->CCR1 = tmp;
		TIM3->CCR2 = tmp;
		TIM3->CCR3 = tmp;
		TIM3->CCR4 = tmp;
		tmp += 25;
		delay_ms(100);
		LED_OFF;
		delay_ms(400);*/
	}
}
