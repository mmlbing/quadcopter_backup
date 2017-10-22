#include "pid.h"

extern MOTOR motor;
extern PID pid;

void QuadPID(void)	//PID
{
	//pitch
	pid.pitchErr = pid.pryPitchCom - pid.pryPitch;
	pid.integralPitch += pid.pitchErr;
	if(pid.integralPitch>100)		pid.integralPitch = 100;
	if(pid.integralPitch<-100)	pid.integralPitch = -100;
	pid.accelXCom = pid.outP*pid.pitchErr + pid.outI*pid.integralPitch + pid.outD*(pid.pitchErr-pid.pitchErrLast);
	pid.pitchErrLast = pid.pitchErr;
	
	pid.accelXErr = pid.accelXCom - pid.accelX;
	pid.integralAccelX += pid.accelXErr;
	if(pid.integralAccelX>100)	pid.integralAccelX = 100;
	if(pid.integralAccelX<-100)	pid.integralAccelX = -100;
	pid.pitchOutput = pid.inP*pid.accelXErr + pid.inI*pid.integralAccelX + pid.inD*(pid.accelXErr-pid.accelXErrLast);
	if(pid.pitchOutput>250)	pid.pitchOutput = 250;
	if(pid.pitchOutput<-250)	pid.pitchOutput = -250;
	pid.accelXErrLast = pid.accelXErr;
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
	pid.accelYCom = pid.outP*pid.rollErr + pid.outI*pid.integralRoll + pid.outD*(pid.rollErr-pid.rollErrLast);
	pid.rollErrLast = pid.rollErr;
	
	pid.accelYErr = pid.accelYCom - pid.accelY;
	pid.integralAccelY += pid.accelYErr;
	if(pid.integralAccelY>100)	pid.integralAccelY = 100;
	if(pid.integralAccelY<-100)	pid.integralAccelY = -100;
	pid.rollOutput = pid.inP*pid.accelYErr + pid.inI*pid.integralAccelY + pid.inD*(pid.accelYErr-pid.accelYErrLast);
	if(pid.rollOutput>250)		pid.rollOutput = 250;
	if(pid.rollOutput<-250)	pid.rollOutput = -250;
	pid.accelYErrLast = pid.accelYErr;
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
	pid.accelZCom = pid.outP*pid.yawErr + pid.outI*pid.integralYaw + pid.outD*(pid.yawErr-pid.yawErrLast);
	pid.yawErrLast = pid.yawErr;
	
	pid.accelZErr = pid.accelZCom - pid.accelZ;
	pid.integralAccelZ += pid.accelZErr;
	if(pid.integralAccelZ>100)	pid.integralAccelZ = 100;
	if(pid.integralAccelZ<-100)	pid.integralAccelZ = -100;
	pid.yawOutput = pid.inP*pid.accelZErr + pid.inI*pid.integralAccelZ + pid.inD*(pid.accelZErr-pid.accelZErrLast);
	if(pid.yawOutput>250)	pid.yawOutput = 250;
	if(pid.yawOutput<-250)	pid.yawOutput = -250;
	pid.accelZErrLast = pid.accelZErr;*/
}
