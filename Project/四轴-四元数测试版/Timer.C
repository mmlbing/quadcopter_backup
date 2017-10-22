#include <STC15W4K60S4.H>
#include <Timer.h>
void Time0_Init()		//8ms@28MHz 定时器0 16位12T自动重载
{
  AUXR &= 0x7F;
	TMOD &= 0xF0;
	IE  = 0x82;
	TL0 = 0x15;
	TH0 = 0xB7;
	TF0 = 0;
	TR0 = 1;
}
void Time2_Init()	//不用在意这个，装逼用的
{
  AUXR |= 0x04;		
	T2L = 0xE5;		
	T2H = 0xFF;	
	AUXR |= 0x10;		
}