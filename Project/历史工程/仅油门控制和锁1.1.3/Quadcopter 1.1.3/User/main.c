#include "stm32f10x.h"
#include "system_init.h"
#include "led.h"
#include "MPU6050.h"
#include "motor.h"
#include "nRF24L01.h"
#include "quadcopter.h"
#include "pid.h"

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
	while(nRF24L01_Check() == 1)
		PrintChar("nRF24L01 check failed!\n");;
	nRF24L01_RX_Mode();
	
	//TIM_Cmd(TIM3, DISABLE);
	
	LoadParameter();
	//清除终端标志和FIFO
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS, 0xFF);
	nRF24L01_Write_Reg(FLUSH_RX,0xFF);
	SystickEnable();
	PrintChar("System running...\n");
	while(1)
	{
	}
}
