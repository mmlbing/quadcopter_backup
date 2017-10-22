/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_sys.h"
#include "etootle_main.h"
//
extern unsigned int system_led_timer_counter,system_timer_1ms_event,system_timer_counter;
unsigned int bs004_sys_timer_period=999;
extern unsigned char BS004_IMU_Output,BS004_Motor_Lock;
//

void BS004_RCC_Configuration(void)		//圆点博士:配置系统时钟
{
  ErrorStatus HSEStartUpStatus;
	//
	NVIC_DeInit();
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

void BS004_NVIC_Configuration(void)				//圆点博士:中断设置
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		//圆点博士:使能串口
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);			//圆点博士:使能电压检测
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);				//圆点博士:使能电压DMA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//圆点博士:使能系统时钟驱动
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);			//圆点博士:使能电机驱动
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//圆点博士:使能SWD下载线
	//
	NVIC_RESETPRIMASK();																			//圆点博士:使能中断
}

void BS004_SYS_LED_Configuration(void)											//圆点博士:LED设置
{
	unsigned char i;
	GPIO_InitTypeDef GPIO_InitStructure;
	//

	GPIO_InitStructure.GPIO_Pin = BS004_Bootloader_USB_CON;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BS004_Bootloader_USB_CON_PORT, &GPIO_InitStructure);
	GPIO_SetBits(BS004_Bootloader_USB_CON_PORT, BS004_Bootloader_USB_CON); 	
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MCU_LED;					//圆点博士:配置使用的LED口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   		//圆点博士:设置LED口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	  	//圆点博士:设置LED口为输出
  GPIO_Init(BS004_MCU_LED_PORT, &GPIO_InitStructure); 
	BS004_MCU_LED_ON();
	//
	for(i=0;i<30;i++)				//圆点博士:快速闪动LED
	{
		BS004_MCU_LED_OFF();
		BS004_Long_Delay(300000);
		BS004_MCU_LED_ON();
		BS004_Long_Delay(300000);
	}
}

void BS004_SYS_Timer_Configuration(void)		
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//
	TIM_TimeBaseStructure.TIM_ClockDivision = BS004_SYS_TIMER_SYSCLK_DIV;
	TIM_TimeBaseStructure.TIM_Prescaler = BS004_SYS_TIMER_CLK_1MHZ;
	TIM_TimeBaseStructure.TIM_Period = bs004_sys_timer_period;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);		//圆点博士:设置PWM周期和频率:72*1K/72M=1mS			
	//
	BS004_SYS_NVIC_Configuration();
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE);					//圆点博士:打开中断
	TIM_Cmd(TIM1, ENABLE);													  //圆点博士:启动PWM
}

void BS004_SYS_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;			//圆点博士:设置PWM中断优先级	
	//
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void BS004_SYS_Timer_Interrupt(void)					//圆点博士:PWM中断函数
{	
	system_timer_1ms_event=1;
}

void BS004_MCU_LED_OFF(void)		
{
	GPIO_SetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);		//圆点博士:熄灭LED
}
void BS004_MCU_LED_ON(void)		
{
	GPIO_ResetBits(BS004_MCU_LED_PORT, BS004_MCU_LED);	 //圆点博士:点亮LED
}
//
void BS004_SYS_LED_TWINKLE(void)										   //圆点博士:闪烁LED
{
	if(system_led_timer_counter>1000) system_led_timer_counter=0;
	if(system_led_timer_counter==500) 
	{
		BS004_MCU_LED_OFF();
		BS004_MOTOR_LED_ON();
	}
	else if(system_led_timer_counter==1000) 
	{
		BS004_MCU_LED_ON();
		BS004_MOTOR_LED_OFF();
	}
	//
	if(system_timer_counter>20) 
	{
		system_timer_counter=0;
		if(BS004_IMU_Output && BS004_Motor_Lock) ANBT_SEND_DMP_EULER_DATA(); 
	}	
}

void BS004_Long_Delay(unsigned int nCount) 		  //圆点博士:延时函数
{
	for(; nCount != 0; nCount--);
}




