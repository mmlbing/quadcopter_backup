#ifndef __STM32_I2C_H
#define __STM32_I2C_H

#include "stm32f10x.h"

#define bool  	 uint8_t
#define TRUE 		 1
#define FALSE		 0

/******************************************/
/**************I2C相关定义*****************/
#define I2CPinGroup		RCC_APB2Periph_GPIOB
#define	I2CGPIO				GPIOB
#define	I2CPin				GPIO_Pin_10 | GPIO_Pin_11			

#define SCL_H         GPIOB->BSRR = GPIO_Pin_10 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOB->BRR  = GPIO_Pin_10 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOB->BSRR = GPIO_Pin_11 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOB->BRR  = GPIO_Pin_11 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

#define	I2C_Direction_Tx   0
#define	I2C_Direction_Rx   1	 


/*====================================================================================================*/
/*====================================================================================================*/
/***********************************************************************************************/
void I2CInit(void);

bool I2CWrite(uint8_t addr, uint8_t reg, uint8_t data);
bool I2CWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
bool I2CReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

/*********************************/
/********MPU6050针对函数**********/
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
/*====================================================================================================*/
/*====================================================================================================*/
#endif
