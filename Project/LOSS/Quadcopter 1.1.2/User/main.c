#include "stm32f10x.h"
#include "system_init.h"
#include "led.h"
#include "MPU6050.h"
#include "motor.h"
#include "nRF24L01.h"
#include "quadcopter.h"

#include "quad_uart.h"

/****************************************
初始化包括：
必须：系统时钟、NVIC、Systick定时器、LED、MPU6050、电机PWM、nRF24L01
可选：串口、
****************************************/
int main()
{
	SysClockInit();
	Initial_UART1(57600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);		//三位抢占--一位响应
	SystickInit();		//未开启
	LEDInit();
	while(MPU6050_Init())
		PrintChar("MPU6050 failed!\n");
	MoterPWMTimerInit();
	nRF24L01_SPI2_Init();
	
	LoadParameter();
	SystickEnable();
	while(1)
	{
		
	}
}
