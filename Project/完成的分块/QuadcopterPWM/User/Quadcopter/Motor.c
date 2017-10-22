#include "motor.h"

/*******************
							M1(P16.T3-1)
					M2+			M4
							M3+
*******************/
struct MOTOR{
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
}motor;

/**
  * @brief  初始化电机的PWM定时器.
	* @detail	输入时钟：=PCLK2=64M
						
	* @resut	设置好后并未开启，可调用TIM_Cmd(TIM3, ENABLE)开启定时器；
						通过TIM3->CCRx来改变占空比.
  * @param  None
  * @retval None
  */
void MoterPWMTimerInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	TIM_OCInitTypeDef  TIM_OCInitStructure;  
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;													//TIMER时钟预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 639;														//自动重装值//计时器最大值==>PWM频率=输入频率/(自动重装值+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//对应通道输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;															//占空比
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;				//默认输出高电平！！！16.7.20验证，别瞎改！
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);		//在其它位置开启
}
