#include "stm32f10x.h"
#include "system_init.h"
#include "STM32_I2C.h"
#include "quad_uart.h"
#include "MPU6050.h"
#include "led.h"

float PRY[3];
short gyro[3], accel[3];

int main()
{
	SysClockInit();
	SystickInit();
	Initial_UART1(57600);
	MPU6050_Init();
	LEDInit();
	LED_ON;
	PrintChar("READY...\n");
	SystickEnable();
	while(1)
	{
		//MPU6050_Get_PRY(PRY, gyro, accel);
		//UART1_ReportIMU(PRY[0],PRY[1],PRY[2],0);
	}
}


/*void delay(uint16_t num)
{
	uint16_t i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<num;j++);
}*/
