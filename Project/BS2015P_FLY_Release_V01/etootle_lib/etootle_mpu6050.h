/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "stm32f10x_lib.h"
#include "etootle_bluetooth.h"
//
#define BS004_MPU6050_INT					GPIO_Pin_3		//PB3
#define BS004_MPU6050_INT_PORT		GPIOB
//
#define BS004_I2C_SDA 			GPIO_Pin_7	 
#define BS004_I2C_SCL 			GPIO_Pin_6	
#define BS004_I2C_PORT   		GPIOB
//
#define BS004_I2C_SCL_0 		GPIO_ResetBits(BS004_I2C_PORT, BS004_I2C_SCL)
#define BS004_I2C_SCL_1 		GPIO_SetBits(BS004_I2C_PORT, BS004_I2C_SCL)
#define BS004_I2C_SDA_0 		GPIO_ResetBits(BS004_I2C_PORT, BS004_I2C_SDA)
#define BS004_I2C_SDA_1   	GPIO_SetBits(BS004_I2C_PORT, BS004_I2C_SDA)
//
#define BS004_I2C_SDA_STATE   	GPIO_ReadInputDataBit(BS004_I2C_PORT, BS004_I2C_SDA)
#define BS004_I2C_DELAY 				BS004_I2C_Delay(100000)
#define BS004_I2C_NOP						BS004_I2C_Delay(10) 
//
#define BS004_I2C_READY					0x00
#define BS004_I2C_BUS_BUSY			0x01	
#define BS004_I2C_BUS_ERROR			0x02
//
#define BS004_I2C_NACK	  0x00 
#define BS004_I2C_ACK			0x01
//
#define BS004_MPU6050_GYRO_ADDR 				0xD0
#define BS004_MPU6050_GYRO_WHOAMI_ADDR 0x75
//
#define BS004_MPU6050_USER_CTRL_ADDR		0x6A
#define BS004_MPU6050_USER_CTRL_VALUE	0x00
//
#define BS004_MPU6050_PWR_MGMT_1_ADDR		0x6B
#define BS004_MPU6050_PWR_MGMT_1_VALUE 	0x01
#define BS004_MPU6050_EXIT_SLEEP_VALUE 	0x01
//
#define BS004_MPU6050_GYRO_CFG_ADDR 		0x1B
#define BS004_MPU6050_GYRO_CFG_VALUE 		0x18
//
#define BS004_MPU6050_ACCEL_CFG_ADDR 		0x1C
#define BS004_MPU6050_ACCEL_CFG_VALUE 	0x18
//
#define BS004_MPU6050_I2CBYPASS_CFG_ADDR 		0x37
#define BS004_MPU6050_I2CBYPASS_CFG_VALUE 	0xB2		//bit7=1, bit6=0, bit5=1, bit4=1, bit3=0, bit2=0, bit1=1, bit0=0
//
#define BS004_MPU6050_INT_CFG_ADDR 		0x38
#define BS004_MPU6050_INT_CFG_VALUE 		0x01
//
#define BS004_MPU6050_ACCEL_DATA_ADDR 		0x3B
//
unsigned char BS004_MPU6050_GYRO_WHOAMI_FUN(void);
unsigned char BS004_MPU6050_READ_REG_FUN(unsigned char bs004_mpu6050_dev_addr,unsigned char bs004_mpu6050_reg_addr);
void BS004_MPU6050_PWM_CFG_FUN(void);
void BS004_MPU6050_GYRO_CFG_FUN(void);
void BS004_MPU6050_ACCEL_CFG_FUN(void);
void BS004_MPU6050_EXIT_SLEEP_FUN(void);
void BS004_MPU6050_I2CBYPASS_CFG_FUN(void);
void BS004_MPU6050_USER_CTRL_FUN(void); 
void BS004_MPU6050_INT_CFG_FUN(void);
void BS004_MPU6050_Init(void);
void BS004_Get_MPU6050_Data(void);
//
void BS004_I2C_Configuration(void);
void BS004_I2C_Delay(unsigned int dly);
unsigned char  BS004_I2C_START(void);
void BS004_I2C_STOP(void);
void BS004_I2C_SendACK(void);
void BS004_I2C_SendNACK(void);
unsigned char  BS004_I2C_SendByte(unsigned char  bs004_i2c_data);
unsigned char  BS004_I2C_ReceiveByte_WithACK(void);
unsigned char  BS004_I2C_ReceiveByte_NoACK(void);
void BS004_I2C_Receive14Bytes(u8 *bs004_i2c_data_buffer);




