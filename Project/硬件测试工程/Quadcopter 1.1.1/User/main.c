#include "stm32f10x.h"
#include "system_init.h"
#include "led.h"
#include "MPU6050.h"
#include "motor.h"
#include "nRF24L01.h"

#include "quad_uart.h"
#include "stdio.h"

/****************************************
初始化包括：
必须：系统时钟、NVIC、Systick定时器、LED、MPU6050、电机PWM、nRF24L01
可选：串口、
****************************************/
int main()
{
	char cdata[10];
	uint16_t whitch=1,clk=0;
	SysClockInit();
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);		//三位抢占--一位响应
	//SystickInit();		//未开启
	LEDInit();
	//MPU6050_Init();
	MoterPWMTimerInit();
	//nRF24L01_SPI2_Init();
	
	Initial_UART1(57600);
	
	while(1)
	{
		switch(whitch){
			case 1:TIM3->CCR1 = clk;PrintChar("M1:");
			break;
			case 2:TIM3->CCR2 = clk;PrintChar("M2:");
			break;
			case 3:TIM3->CCR3 = clk;PrintChar("M3:");
			break;
			case 4:TIM3->CCR4 = clk;PrintChar("M4:");
			break;
			default:whitch = 1;
			break;
		}
		clk += 5;
		if(clk==640)
		{
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = 0;
			TIM3->CCR4 = 0;
			whitch++;
			if(whitch == 5)
				whitch =1;
			clk = 0;
		}
		sprintf(cdata,"%d",clk);
		PrintChar(cdata);
		PrintChar("\n");
		delay_ms(300);
	}
}
