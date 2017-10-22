/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

extern char SysTickFlow;

extern uint32_t IntNum;					//5ms中断次数技术
extern uint8_t RxData[11];			//24L01收到的原始数据

extern uint8_t LockState;

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

extern struct MOTOR{
	int16_t motor_1;
	int16_t motor_2;
	int16_t motor_3;
	int16_t motor_4;
}motor;

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	ControlPrepare();			//准备控制数据
	MPU6050_Get_PRY();		//获取实际数据
	if(LockState == UNLOCKED)
	{
		//QuadPID();						//串级PID
		QuadPIDA();						//角度PID
		TimerPWMControl();		//改变PWM
		//PrintChar("UNLOCKED\n");
		//UAR1_ReportMotor();
	}
	//else
		//PrintChar("LOCKED\n");
	if(!(IntNum%100))
	{
		LEDChangeState();
		UAR1_ReportPID();
		UAR1_ReportMotor();
		UAR1_ReportPRY();
		PrintChar("\n");
	}
	//UART1_ReportIMU(pid.pryPitch,pid.pryRoll,pid.pryYaw,0);
	//UART1_ReportGyro(pid.gyroX, pid.gyroY, pid.gyroZ);
	
	IntNum++;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line11);
	nRF24L01_RxPacket(RxData);
	if(RxData[0] == 0xAA)	//Command,else control data
	{
		if(LockState == LOCKED)
		{
			LockState = UNLOCKED;
		}
		else
		{
			LockState = LOCKED;
			PIDClear();
			motor.motor_1 = 0;
			motor.motor_2 = 0;
			motor.motor_3 = 0;
			motor.motor_4 = 0;
			TimerPWMControl();
		}
	}
}

void TIM3_IRQHandler(void)
{
	
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
