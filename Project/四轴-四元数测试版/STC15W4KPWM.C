#include <STC15W4K60S4.H>
#include <STC15W4KPWM.H>
#include <NRF24L01.H>
#include <Timer.h>
void PWMGO()
{
	//所有I/O口全设为准双向，弱上拉模式
	P0M0=0x00;
	P0M1=0x00;
	P1M0=0x00;
	P1M1=0x00;
	P2M0=0x00;      
	P2M1=0x00;
	P3M0=0x00;
	P3M1=0x00;
	P4M0=0x00;
	P4M1=0x00;
	P5M0=0x00;
	P5M1=0x00;
	P6M0=0x00;
	P6M1=0x00;
	P7M0=0x00;
	P7M1=0x00;
	//设置需要使用的PWM输出口为强推挽模式
	P2M0=0x0e;
	P2M1=0x00;
	P3M0=0x80;
	P3M1=0x00;
	//使用定时器2作为时钟源
	Time2_Init();
	
	P_SW2=0x80;    //最高位置1才能访问和PWM相关的特殊寄存器
	
  PWMCFG=0xb0;    //7位    6位                5位    4位    3位    2位    1位    0位 
	                //置0  1-计数器归零触发ADC C7INI  C6INI  C5INI  C4INI  C3INI  C2INI
	                //     0-归零时不触发ADC       (值为1时上电高电平，为0低电平）   
	
	PWMCKS=0x10;    //7位6位5位    4位             3位    2位    1位    0位 
	                //   置0    0-系统时钟分频          分频参数设定
	                //          1-定时器2溢出       时钟=系统时钟/([3:0]+1)
	
	PWMIF=0x00;     //7位    6位                5位    4位    3位    2位    1位    0位 
                  //置0  计数器归零中断标志            相应PWM端口中断标志
	
	PWMFDCR=0x00;   //7位    6位       5位                4位     
	                //置0    置0 外部异常检测开关  外部异常时0-无反应 1-高阻状态
	                //3位             2位                 1位                0位      
                  //PWM异常中断  比较器与异常的关系   P2.4与异常的关系  PWM异常标志

  PWMCH=0x03;     //15位寄存器，决定PWM周期，数值为1-32767，单位：脉冲时钟
  PWMCL=0xe9;
	
// 以下为每个PWM输出口单独设置
	PWM2CR=0x00;    //7位6位5位4位   3位      2位       1位        0位 
	                //    置0      输出切换 中断开关 T2中断开关 T1中断开关
	PWM3CR=0x00; 
	PWM4CR=0x00; 
	PWM5CR=0x00; 
	
	PWM2T1H=0x03;          //15位寄存器第一次翻转计数  第一次翻转是指从低电平翻转到高电平的计时
	PWM2T1L=0xe8;
	PWM2T2H=0x03;          //15位寄存器第二次翻转计数  第二次翻转是指从高电平翻转到低电平的计时
	PWM2T2L=0xe9;          //第二次翻转应比精度等级要高，否则会工作不正常，比如精度1000，第二次翻转就必须小于1000
	  
	PWM3T1H=0x03;
	PWM3T1L=0xe8;
	PWM3T2H=0x03;  
	PWM3T2L=0xe9;
	  
	PWM4T1H=0x03;
  PWM4T1L=0xe8;
	PWM4T2H=0x03;  
	PWM4T2L=0xe9;
	 
	PWM5T1H=0x03;
	PWM5T1L=0xe8;
	PWM5T2H=0x03;  
	PWM5T2L=0xe9;
//以上为每个PWM输出口单独设置

  PWMCR=0x8f;     //7位          6位                5位 4位 3位 2位 1位 0位     10001111
	                //PWM开关 计数归零中断开关   相应I/O为GPIO模式(0)或PWM模式(1)								
//*********************************以下为装逼模拟电调的声音******************************************************
	PWM(960,960,960,960);
	Delay(60000);Delay(60000);Delay(60000);
	
	T2L = 0xE8;		
	T2H = 0xFF;		
	Delay(60000);Delay(60000);Delay(60000);
	
	T2L = 0xEB;	
	T2H = 0xFF;
	Delay(60000);Delay(60000);Delay(60000);
	
	PWM(1000,1000,1000,1000);
	Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);
	
	T2L = 0xE5;		
	T2H = 0xFF;	
	PWM(960,960,960,960);
	Delay(60000);Delay(60000);Delay(60000);
	PWM(1000,1000,1000,1000);
	
	Delay(60000);Delay(60000);Delay(60000);
	PWM(960,960,960,960);
	Delay(60000);Delay(60000);Delay(60000);
	PWM(1000,1000,1000,1000);
	
	Delay(60000);Delay(60000);Delay(60000);
	PWM(960,960,960,960);
	Delay(60000);Delay(60000);Delay(60000);
	PWM(1000,1000,1000,1000);
	
	Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);
	PWM(960,960,960,960);
	Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);Delay(60000);
	PWM(1000,1000,1000,1000);
//**************************************以上为装逼模拟电调的声音**************************************************
	PWMCKS=0x00;
}
//本函数输入的4个值取值范围为0-1000 1000电机停，0转速最高，输入数据不能超过取值范围，否则会大姨妈
void PWM(unsigned int PWMa,unsigned int PWMb,unsigned int PWMc,unsigned int PWMd)
{  
		PWM2T1H=PWMa>>8;          //15位寄存器第一次翻转计数  第一次翻转是指从低电平翻转到高电平的计时
		PWM2T1L=PWMa;
	
	  PWM3T1H=PWMb>>8;
		PWM3T1L=PWMb;

	  PWM4T1H=PWMc>>8;
		PWM4T1L=PWMc;

	  PWM5T1H=PWMd>>8;
		PWM5T1L=PWMd;
}