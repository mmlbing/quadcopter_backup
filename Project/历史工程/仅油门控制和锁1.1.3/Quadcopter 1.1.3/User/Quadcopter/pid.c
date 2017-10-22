#include "pid.h"

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

void QuadPID(void)	//PID
{
	//pitch
	pid.pitchErr = pid.pryPitchCom - pid.pryPitch;
	pid.integralPitch += pid.pitchErr;
	if(pid.integralPitch>100)		pid.integralPitch = 100;
	if(pid.integralPitch<-100)	pid.integralPitch = -100;
	pid.gyroXCom = pid.outP*pid.pitchErr + pid.outI*pid.integralPitch + pid.outD*(pid.pitchErr-pid.pitchErrLast);
	pid.pitchErrLast = pid.pitchErr;
	
	pid.gyroXErr = pid.gyroXCom - pid.gyroX;
	pid.integralgyroX += pid.gyroXErr;
	if(pid.integralgyroX>100)	pid.integralgyroX = 100;
	if(pid.integralgyroX<-100)	pid.integralgyroX = -100;
	pid.pitchOutput = pid.inP*pid.gyroXErr + pid.inI*pid.integralgyroX + pid.inD*(pid.gyroXErr-pid.gyroXErrLast);
	if(pid.pitchOutput>250)	pid.pitchOutput = 250;
	if(pid.pitchOutput<-250)	pid.pitchOutput = -250;
	pid.gyroXErrLast = pid.gyroXErr;
	if(pid.pitchOutput>0)
	{
		motor.motor_2 += pid.pitchOutput;
		motor.motor_3 += pid.pitchOutput;
	}
	else
	{
		motor.motor_1 -= pid.pitchOutput;
		motor.motor_4 -= pid.pitchOutput;
	}
	
	//Roll
	pid.rollErr = pid.pryRollCom - pid.pryRoll;
	pid.integralRoll += pid.rollErr;
	if(pid.integralRoll>100)	pid.integralRoll = 100;
	if(pid.integralRoll<-100)	pid.integralRoll = -100;
	pid.gyroYCom = pid.outP*pid.rollErr + pid.outI*pid.integralRoll + pid.outD*(pid.rollErr-pid.rollErrLast);
	pid.rollErrLast = pid.rollErr;
	
	pid.gyroYErr = pid.gyroYCom - pid.gyroY;
	pid.integralgyroY += pid.gyroYErr;
	if(pid.integralgyroY>100)	pid.integralgyroY = 100;
	if(pid.integralgyroY<-100)	pid.integralgyroY = -100;
	pid.rollOutput = pid.inP*pid.gyroYErr + pid.inI*pid.integralgyroY + pid.inD*(pid.gyroYErr-pid.gyroYErrLast);
	if(pid.rollOutput>250)		pid.rollOutput = 250;
	if(pid.rollOutput<-250)	pid.rollOutput = -250;
	pid.gyroYErrLast = pid.gyroYErr;
	if(pid.rollOutput>0)
	{
		motor.motor_3 += pid.rollOutput;
		motor.motor_4 += pid.rollOutput;
	}
	else
	{
		motor.motor_1 -= pid.rollOutput;
		motor.motor_2 -= pid.rollOutput;
	}
	
	//Yaw
	/*pid.yawErr = pid.pryYawCom - pid.pryYaw;
	pid.integralYaw += pid.yawErr;
	if(pid.integralYaw>100)		pid.integralYaw = 100;
	if(pid.integralYaw<-100)	pid.integralYaw = -100;
	pid.gyroZCom = pid.outP*pid.yawErr + pid.outI*pid.integralYaw + pid.outD*(pid.yawErr-pid.yawErrLast);
	pid.yawErrLast = pid.yawErr;
	
	pid.gyroZErr = pid.gyroZCom - pid.gyroZ;
	pid.integralgyroZ += pid.gyroZErr;
	if(pid.integralgyroZ>100)	pid.integralgyroZ = 100;
	if(pid.integralgyroZ<-100)	pid.integralgyroZ = -100;
	pid.yawOutput = pid.inP*pid.gyroZErr + pid.inI*pid.integralgyroZ + pid.inD*(pid.gyroZErr-pid.gyroZErrLast);
	if(pid.yawOutput>250)	pid.yawOutput = 250;
	if(pid.yawOutput<-250)	pid.yawOutput = -250;
	pid.gyroZErrLast = pid.gyroZErr;*/
}
