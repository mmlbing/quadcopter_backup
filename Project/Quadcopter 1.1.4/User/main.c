#include "stm32f10x.h"
#include "system_init.h"
#include "led.h"
#include "MPU6050.h"
#include "motor.h"
#include "nRF24L01.h"
#include "quadcopter.h"
#include "pid.h"

#include "quad_uart.h"
#include <stdlib.h>

extern char UART1RxData[50];
extern char pidFresh;
extern struct PID{
	float pryPitch,pryRoll,pryYaw;								//欧拉角实际值（外环）
	float pryPitchCom,pryRollCom,pryYawCom;				//欧拉角要求值（外环）
	float pitchErr,rollErr,yawErr;								//欧拉角当前偏差
	float pitchErrLast,rollErrLast,yawErrLast;		//欧拉角上次偏差
	float outP,outI,outD;													//外环PID参数
	float integralPitch,integralRoll,integralYaw;	//外环欧拉角积分累计
	
	float gyroX,gyroY,gyroZ;															//角速度当前值（内环）
	float gyroXCom,gyroYCom,gyroZCom;											//角速度要求值（内环）//外环输出！
	float gyroXErr,gyroYErr,gyroZErr;											//角速度当前偏差
	float gyroXErrLast,gyroYErrLast,gyroZErrLast;					//角速度上次偏差
	float inP,inI,inD;																		//内环PID参数
	float integralgyroX,integralgyroY,integralgyroZ;			//内环角速度积分累计
	
	float pitchOutput,rollOutput,yawOutput;				//最终输出
}pid;

/****************************************
初始化包括：
必须：系统时钟、NVIC、Systick定时器、LED、MPU6050、电机PWM、nRF24L01
可选：串口、
****************************************/
int main()
{
	/*SysClockInit();
	Initial_UART1(57600);
	PrintChar("temp:21.0    fan-state:stop\n");
	PrintChar("temp:22.3    fan-state:stop\n");
	PrintChar("temp:25.2    fan-state:slow\n");
	PrintChar("temp:28.7    fan-state:fast\n");*/
	char tmp[5],i;
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
		if(pidFresh)
		{
			for(i=0;i<5;i++)
				tmp[i] = UART1RxData[i];
			pid.outP = atof(tmp);
			
			for(i=0;i<5;i++)
				tmp[i] = UART1RxData[i+6];
			pid.outI = atof(tmp);
			
			for(i=0;i<5;i++)
				tmp[i] = UART1RxData[i+12];
			pid.outD = atof(tmp);
			pidFresh = 0;
		}
	}
}
