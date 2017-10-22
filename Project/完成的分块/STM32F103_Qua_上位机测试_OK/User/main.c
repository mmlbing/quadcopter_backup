#include "stm32f10x.h"
#include "quad_uart.h"
//#include "rcc_config.c"
//#include "system_stm32f10x.h"

//extern void SetSysClock(void);
void delay(uint16_t num)
{
	uint16_t i,j;
	for(i=0;i<1000;i++)
		for(j=0;j<num;j++);
}

int main()
{
	float yaw = 0, pitch = 0, roll = 0, tempr = 10;
	SystemInit();
	Initial_UART1(57600);
	while(1)
	{
		//UART1_Put_Char(0xD3);
		UART1_ReportIMU(yaw, pitch, roll, tempr);
		yaw += 0.1;
		pitch += 0.1;
		roll += 0.1;
		tempr += 1;
		if(yaw == 2.5)
		{
			yaw = 0;
			pitch = 0;
			roll = 0;
			tempr = 10;
		}
		delay(100);
	}
}
