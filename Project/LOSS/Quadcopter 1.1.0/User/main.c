#include "stm32f10x.h"
#include "system_init.h"
#include "led.h"
#include "MPU6050.h"
#include "moter.h"
#include "nRF24L01.h"

#include "quad_uart.h"

extern GPIO_InitTypeDef GPIO_InitStructures;

/****************************************
初始化包括：
必须：系统时钟、NVIC、Systick定时器、LED、MPU6050、电机PWM、nRF24L01
可选：串口、
****************************************/
int main()
{
	GPIO_InitStructures.GPIO_Pin =  GPIO_Pin_8; 
  GPIO_InitStructures.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructures.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOC, &GPIO_InitStructures);
	//SysClockInit();
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);		//三位抢占--一位响应
	//SystickInit();		//未开启
	//LEDInit();
	//MPU6050_Init();
	MoterPWMTimerInit();
	nRF24L01_SPI2_Init();
	
	//Initial_UART1(57600);
	
	while(1)
	{
	}
}
