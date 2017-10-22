#include "pida.h"

extern struct MOTOR{
	int16_t motor_1;
	int16_t motor_2;
	int16_t motor_3;
	int16_t motor_4;
}motor;

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

void QuadPIDA(void)	//PID
{
	//pitch
	/*pid.pitchErr = pid.pryPitchCom - pid.pryPitch;
	pid.integralPitch += pid.pitchErr;
	if(pid.integralPitch>300)		pid.integralPitch = 300;
	if(pid.integralPitch<-300)	pid.integralPitch = -300;
	pid.pitchOutput = pid.outP*pid.pitchErr + pid.outI*pid.integralPitch + pid.outD*(pid.pitchErr-pid.pitchErrLast);
	pid.pitchErrLast = pid.pitchErr;
	
		motor.motor_2 += pid.pitchOutput;
		motor.motor_3 += pid.pitchOutput;
		motor.motor_1 -= pid.pitchOutput;
		motor.motor_4 -= pid.pitchOutput;*/
	
	//Roll
	pid.rollErr = pid.pryRollCom - pid.pryRoll;
	pid.integralRoll += pid.rollErr;
	if(pid.integralRoll>500)	pid.integralRoll = 500;
	if(pid.integralRoll<-500)	pid.integralRoll = -500;
	pid.rollOutput = pid.outP*pid.rollErr + pid.outI*pid.integralRoll + pid.outD*(pid.rollErr-pid.rollErrLast);
	pid.rollErrLast = pid.rollErr;
	

		motor.motor_1 += pid.rollOutput;
		motor.motor_2 += pid.rollOutput;
		motor.motor_3 -= pid.rollOutput;
		motor.motor_4 -= pid.rollOutput;
}

void PIDClear(void)
{
	pid.pitchErr = 0;
	pid.rollErr = 0;
	pid.yawErr = 0;
	pid.pitchErrLast = 0;
	pid.rollErrLast = 0;
	pid.yawErrLast = 0;
	pid.integralPitch = 0;
	pid.integralRoll = 0;
	pid.integralYaw = 0;
	pid.pitchOutput = 0;
	pid.rollOutput = 0;
}
