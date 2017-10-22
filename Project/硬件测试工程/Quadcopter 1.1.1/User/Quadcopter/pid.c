#include "pid.h"

struct PID{
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
}pid;

void QuadPID(void)	//PID
{
	//pitch
	pid.pitchErr = pid.pryPitchCom - pid.pryPitch;
	pid.integralPitch += pid.pitchErr;
	if(pid.integralPitch>500)		pid.integralPitch = 500;
	if(pid.integralPitch<-500)	pid.integralPitch = -500;
	pid.accelXCom = pid.outP*pid.pitchErr + pid.outI*pid.integralPitch + pid.outD*(pid.pitchErr-pid.pitchErrLast);
	pid.pitchErrLast = pid.pitchErr;
	
	pid.accelXErr = pid.accelXCom - pid.accelX;
	pid.integralAccelX += pid.accelXErr;
	if(pid.integralAccelX>500)	pid.integralAccelX = 500;
	if(pid.integralAccelX<-500)	pid.integralAccelX = -500;
	pid.pitchOutput = pid.inP*pid.accelXErr + pid.inI*pid.integralAccelX + pid.inD*(pid.accelXErr-pid.accelXErrLast);
	if(pid.pitchOutput>4000)	pid.pitchOutput = 4000;
	if(pid.pitchOutput<-4000)	pid.pitchOutput = -4000;
	pid.accelXErrLast = pid.accelXErr;
	if(pid.pitchOutput>0)
		motor.motor_3 += pid.pitchOutput;
	else
		motor.motor_1 -= pid.pitchOutput;
	
	//Roll
	pid.rollErr = pid.pryRollCom - pid.pryRoll;
	pid.integralRoll += pid.rollErr;
	if(pid.integralRoll>500)	pid.integralRoll = 500;
	if(pid.integralRoll<-500)	pid.integralRoll = -500;
	pid.accelYCom = pid.outP*pid.rollErr + pid.outI*pid.integralRoll + pid.outD*(pid.rollErr-pid.rollErrLast);
	pid.rollErrLast = pid.rollErr;
	
	pid.accelYErr = pid.accelYCom - pid.accelY;
	pid.integralAccelY += pid.accelYErr;
	if(pid.integralAccelY>500)	pid.integralAccelY = 500;
	if(pid.integralAccelY<-500)	pid.integralAccelY = -500;
	pid.rollOutput = pid.inP*pid.accelYErr + pid.inI*pid.integralAccelY + pid.inD*(pid.accelYErr-pid.accelYErrLast);
	if(pid.rollOutput>4000)		pid.rollOutput = 4000;
	if(pid.rollOutput<-4000)	pid.rollOutput = -4000;
	pid.accelYErrLast = pid.accelYErr;
	if(pid.rollOutput>0)
		motor.motor_4 += pid.rollOutput;
	else
		motor.motor_2 -= pid.rollOutput;
	
	//Yaw
	/*pid.yawErr = pid.pryYawCom - pid.pryYaw;
	pid.integralYaw += pid.yawErr;
	if(pid.integralYaw>500)		pid.integralYaw = 500;
	if(pid.integralYaw<-500)	pid.integralYaw = -500;
	pid.accelZCom = pid.outP*pid.yawErr + pid.outI*pid.integralYaw + pid.outD*(pid.yawErr-pid.yawErrLast);
	pid.yawErrLast = pid.yawErr;
	
	pid.accelZErr = pid.accelZCom - pid.accelZ;
	pid.integralAccelZ += pid.accelZErr;
	if(pid.integralAccelZ>500)	pid.integralAccelZ = 500;
	if(pid.integralAccelZ<-500)	pid.integralAccelZ = -500;
	pid.yawOutput = pid.inP*pid.accelZErr + pid.inI*pid.integralAccelZ + pid.inD*(pid.accelZErr-pid.accelZErrLast);
	if(pid.yawOutput>4000)	pid.yawOutput = 4000;
	if(pid.yawOutput<-4000)	pid.yawOutput = -4000;
	pid.accelZErrLast = pid.accelZErr;*/
}
