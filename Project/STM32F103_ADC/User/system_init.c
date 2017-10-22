#include "system_init.h"

/**
  * @brief  设置系统时钟.
	* @detail	外部晶振8M，PLL9倍频，72MHz
						SYSCLK = 72M
						HCLK = SYSCLK = 72M
						PCLK1 = HCLK = 72M
						PCLK2 = HCLK/2 = 36M
  * @param  None
  * @retval None
  */
void SysClockInit(void)
{
	uint8_t HSEStartUpStatus;
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  HSEStartUpStatus = RCC_WaitForHSEStartUp();
  if(HSEStartUpStatus == SUCCESS)
  {
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
		
    //FLASH_SetLatency(FLASH_Latency_2);
    //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
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

/**
  * @brief  设置Systick定时器，不使能
	* @detail	时钟源：HCLK/8 = 9MHz
  * @param  None
  * @retval None
  */
void SystickInit(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	SysTick->LOAD = 24000;
	NVIC_SetPriority(SysTick_IRQn, 0x00);
	SysTick->VAL = 0; 
	SysTick->CTRL |= 0x02;
}

/**
  * @brief  使能Systick定时器.
	* @detail	
  * @param  None
  * @retval None
  */
void SystickEnable(void)
{
	SysTick->CTRL |= 0x01;
}
