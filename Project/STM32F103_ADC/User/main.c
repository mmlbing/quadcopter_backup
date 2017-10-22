#include "stm32f10x.h"
#include "system_init.h"
#include "handleAD.h"

int main()
{
	SysClockInit();
	ADC12_Init();
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(1)
	{
	}
}
