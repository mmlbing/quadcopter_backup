#include "stm32f10x.h"
#include "stdio.h"

uint32_t IntNum = 0;		//5ms中断次数技术
uint8_t RxData[11];			//24L01收到的原始数据

/*********************************
							M1(P16.T3-1)
					M2+			M4
							M3+
*********************************/
typedef struct {
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
}MOTOR;

typedef struct {
	float pryPitch,pryRoll,pryYaw;								//欧拉角实际值（外环）
	float pryPitchCom,pryRollCom,pryYawCom;				//欧拉角要求值（外环）
	float pitchErr,rollErr,yawErr;								//欧拉角当前偏差
	float pitchErrLast,rollErrLast,yawErrLast;		//欧拉角上次偏差
	float outP,outI,outD;													//外环PID参数
	float integralPitch,integralRoll,integralYaw;	//外环欧拉角积分累计
	
	float accelX,accelY,accelZ;															//角速度当前值（内环）
	float accelXCom,accelYCom,accelZCom;										//角速度要求值（内环）//外环输出！
	float accelXErr,accelYErr,accelZErr;										//角速度当前偏差
	float accelXErrLast,accelYErrLast,accelZErrLast;				//角速度上次偏差
	float inP,inI,inD;																			//内环PID参数
	float integralAccelX,integralAccelY,integralAccelZ;			//内环角速度积分累计
	
	float pitchOutput,rollOutput,yawOutput;			//最终输出
}PID;

void ControlPrepare(void);
void LoadParameter(void);
