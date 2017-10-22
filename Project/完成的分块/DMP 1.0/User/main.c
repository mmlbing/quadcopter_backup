#include "stm32f10x.h"
#include "STM32_I2C.h"
#include "quad_uart.h"
#include "MPU6050.h"


int main()
{
	float PRY[3];
	
	SystemInit();
	Initial_UART1(57600);
	i2cInit();
	while(MPU6050_Init())
		PrintChar("MPU6050 failed!\n");
	
	while(1)
	{
		MPU6050_Get_PRY(PRY);
		UART1_ReportIMU(PRY[0],PRY[1],PRY[2],0);
	}
}


/*void delay(uint16_t num)
{
	uint16_t i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<num;j++);
}*/
