#include "quadcopter.h"

uint32_t IntNum = 0;		//5ms中断次数技术
uint8_t RxData[11];			//24L01收到的原始数据

int16_t Power = 0, PowerLast = 0, PowerErr = 0;
uint8_t LockState = LOCKED;

/*********************************
							M1(P16.T3-1)
					M2+			M4
							M3+
*********************************/
struct MOTOR{
	int16_t motor_1;
	int16_t motor_2;
	int16_t motor_3;
	int16_t motor_4;
}motor={0,0,0,0};

struct PID{
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
}pid={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*****函数******************************************************************************************************************************/
/**
  * @brief  将收到的原始控制数据进行转换.
	* @detail	将RxData[]的数据转换并存到pid.pryPitchCom,pryRollCom,pryYawCom中
	* @resut	
  * @param  None
  * @retval None
  */
void ControlPrepare(void)
{
	if(RxData[0] == 0xA5)	//收到角度控制数据
	{
		Power = (int16_t)( (RxData[1]<<8)|RxData[2] );
		pid.pryPitchCom = ((float)(int16_t)( (RxData[3]<<8)|RxData[4] ))/10;
		pid.pryRollCom = ((float)(int16_t)( (RxData[5]<<8)|RxData[6] ))/10;
		pid.pryYawCom = ((float)(int16_t)( (RxData[7]<<8)|RxData[8] ))/10;
		
		PowerErr = Power - PowerLast;	//油门位移量转化为油门差值
		PowerLast = Power;
		
		motor.motor_1 += PowerErr;		//将油门差值加上
		motor.motor_2 += PowerErr;
		motor.motor_3 += PowerErr;
		motor.motor_4 += PowerErr;
		if(motor.motor_1 < 0)
			motor.motor_1 = 0;
		if(motor.motor_2 < 0)
			motor.motor_2 = 0;
		if(motor.motor_3 < 0)
			motor.motor_3 = 0;
		if(motor.motor_4 < 0)
			motor.motor_4 = 0;
	}
}

/**
  * @brief  装载参数.
	* @detail	装载pid参数，给变量赋初值等
	* @resut	
  * @param  None
  * @retval None
  */
void LoadParameter(void)
{
	pid.outP = 1;
	pid.outI = 0;
	pid.outD = 0;
	pid.inP = 0;
	pid.inI = 0;
	pid.inD = 0;
}
