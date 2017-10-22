/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_mpu6050.h"
#include "etootle_sys.h"
//
signed short int bs004_mpu6050_gyro_pitch_raw=0,bs004_mpu6050_gyro_roll_raw=0,bs004_mpu6050_gyro_yaw_raw=0;
signed short int bs004_mpu6050_gyro_pitch_cal=0,bs004_mpu6050_gyro_roll_cal=0,bs004_mpu6050_gyro_yaw_cal=0;
signed short int bs004_mpu6050_acc_pitch_raw=0,bs004_mpu6050_acc_roll_raw=0,bs004_mpu6050_acc_yaw_raw=0;
signed short int bs004_mpu6050_acc_pitch_cal=0,bs004_mpu6050_acc_roll_cal=0,bs004_mpu6050_acc_yaw_cal=0;
signed short int bs004_hmc5883l_mag_pitch_cal=0,bs004_hmc5883l_mag_roll_cal=0,bs004_hmc5883l_mag_yaw_cal=0;
//
float  bs004_filter_high=0,bs004_filter_low=0,bs004_filter_time=0;
float  bs004_mpu6050_acc_pitch_com=0,bs004_mpu6050_acc_roll_com=0;
//
float bs004_mpu6050_gyro_angel_pitch_ave=0,bs004_mpu6050_gyro_angel_roll_ave=0,bs004_mpu6050_gyro_angel_yaw_ave=0;
float bs004_mpu6050_acc_angel_pitch_ave=0,bs004_mpu6050_acc_angel_roll_ave=0,bs004_mpu6050_acc_angel_yaw_ave=0;
//
void BS004_Get_MPU6050_Data(void)   
{
	unsigned char i;
	unsigned char bs004_mpu6050_data_buffer[14];
	signed short int bs004_mpu6050_raw_data[7];
	//
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_DATA_ADDR);    //圆点博士:发送陀螺仪寄存器地址
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      	//圆点博士:发送陀螺仪读地址
	BS004_I2C_Receive14Bytes(bs004_mpu6050_data_buffer);		//圆点博士:读出陀螺仪寄存器值
	BS004_I2C_STOP();
	//				
	for(i=0;i<7;i++) bs004_mpu6050_raw_data[i]=(((signed short int)bs004_mpu6050_data_buffer[i*2]) << 8) | bs004_mpu6050_data_buffer[i*2+1];
	bs004_mpu6050_acc_roll_raw=bs004_mpu6050_raw_data[0]-bs004_mpu6050_acc_roll_cal;
	bs004_mpu6050_acc_pitch_raw=bs004_mpu6050_raw_data[1]-bs004_mpu6050_acc_pitch_cal;
	bs004_mpu6050_acc_yaw_raw=bs004_mpu6050_raw_data[2];
	bs004_mpu6050_gyro_pitch_raw=bs004_mpu6050_raw_data[4]-bs004_mpu6050_gyro_pitch_cal;
	bs004_mpu6050_gyro_roll_raw=bs004_mpu6050_raw_data[5]-bs004_mpu6050_gyro_roll_cal;;
	bs004_mpu6050_gyro_yaw_raw=bs004_mpu6050_raw_data[6]-bs004_mpu6050_gyro_yaw_cal;
	//	
	bs004_mpu6050_acc_pitch_com=bs004_filter_high*bs004_mpu6050_acc_pitch_com+bs004_filter_low*bs004_mpu6050_acc_pitch_raw;	
	bs004_mpu6050_acc_roll_com =bs004_filter_high*bs004_mpu6050_acc_roll_com +bs004_filter_low*bs004_mpu6050_acc_roll_raw;
	//
  bs004_mpu6050_acc_angel_pitch_ave=(bs004_mpu6050_acc_angel_pitch_ave+bs004_mpu6050_acc_pitch_com)/2.0f;
	bs004_mpu6050_acc_angel_roll_ave =(bs004_mpu6050_acc_angel_roll_ave +bs004_mpu6050_acc_roll_com)/2.0f;
	bs004_mpu6050_acc_angel_yaw_ave  =(bs004_mpu6050_acc_angel_yaw_ave  +bs004_mpu6050_acc_yaw_raw)/2.0f;
	//
	bs004_mpu6050_gyro_angel_pitch_ave=(bs004_mpu6050_gyro_angel_pitch_ave+bs004_mpu6050_gyro_pitch_raw)/2.0f;
	bs004_mpu6050_gyro_angel_roll_ave =(bs004_mpu6050_gyro_angel_roll_ave +bs004_mpu6050_gyro_roll_raw)/2.0f;
	bs004_mpu6050_gyro_angel_yaw_ave  =(bs004_mpu6050_gyro_angel_yaw_ave  +bs004_mpu6050_gyro_yaw_raw)/2.0f;
}
//
void BS004_I2C_Configuration(void)			
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//
  GPIO_InitStructure.GPIO_Pin = BS004_I2C_SCL | BS004_I2C_SDA;	//圆点博士:配置使用的I2C口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   						//圆点博士:设置I2C口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	  					//圆点博士:设置I2C为开漏输出
  GPIO_Init(BS004_I2C_PORT, &GPIO_InitStructure); 
	//
	GPIO_InitStructure.GPIO_Pin = BS004_MPU6050_INT;					//圆点博士:配置使用的I2C口
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   				//圆点博士:设置I2C口最大允许输出速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  	//圆点博士:设置I2C为开漏输出
  GPIO_Init(BS004_MPU6050_INT_PORT, &GPIO_InitStructure); 
	//
	BS004_I2C_SCL_1; 
	BS004_I2C_SDA_1;
	BS004_I2C_DELAY;
}

void BS004_I2C_Delay(unsigned int dly)               
{
	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

unsigned char BS004_I2C_START(void)
{ 
	BS004_I2C_SDA_1; 
 	BS004_I2C_NOP;
  // 
 	BS004_I2C_SCL_1; 
 	BS004_I2C_NOP;    
	//
 	if(!BS004_I2C_SDA_STATE) return BS004_I2C_BUS_BUSY;
	//
 	BS004_I2C_SDA_0;
 	BS004_I2C_NOP;
  //
 	BS004_I2C_SCL_0;  
 	BS004_I2C_NOP; 
	//
 	if(BS004_I2C_SDA_STATE) return BS004_I2C_BUS_ERROR;
	//
 	return BS004_I2C_READY;
}

void BS004_I2C_STOP(void)
{
 	BS004_I2C_SDA_0; 
 	BS004_I2C_NOP;
  // 
 	BS004_I2C_SCL_1; 
 	BS004_I2C_NOP;    
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_NOP;
}

void BS004_I2C_SendACK(void)
{
 	BS004_I2C_SDA_0;
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_1;
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_0; 
 	BS004_I2C_NOP;  
}

void BS004_I2C_SendNACK(void)
{
	BS004_I2C_SDA_1;
	BS004_I2C_NOP;
	BS004_I2C_SCL_1;
	BS004_I2C_NOP;
	BS004_I2C_SCL_0; 
	BS004_I2C_NOP;  
}

unsigned char BS004_I2C_SendByte(u8 bs004_i2c_data)
{
 	unsigned char i;
 	//
	BS004_I2C_SCL_0;
 	for(i=0;i<8;i++)
 	{  
  		if(bs004_i2c_data&0x80) BS004_I2C_SDA_1;
   		else BS004_I2C_SDA_0;
			//
  		bs004_i2c_data<<=1;
  		BS004_I2C_NOP;
			//
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP;
  		BS004_I2C_SCL_0;
  		BS004_I2C_NOP; 
 	}
	//
 	BS004_I2C_SDA_1; 
 	BS004_I2C_NOP;
 	BS004_I2C_SCL_1;
 	BS004_I2C_NOP;   
 	if(BS004_I2C_SDA_STATE)
 	{
  		BS004_I2C_SCL_0;
  		return BS004_I2C_NACK;
 	}
 	else
 	{
  		BS004_I2C_SCL_0;
  		return BS004_I2C_ACK;  
 	}    
}

unsigned char BS004_I2C_ReceiveByte_NoACK(void)
{
	unsigned char i,bs004_i2c_data;
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_SCL_0; 
 	bs004_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		bs004_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	bs004_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
 	}
	BS004_I2C_SendNACK();
 	return bs004_i2c_data;
}

unsigned char BS004_I2C_ReceiveByte_WithACK(void)
{
	unsigned char i,bs004_i2c_data;
	//
 	BS004_I2C_SDA_1;
 	BS004_I2C_SCL_0; 
 	bs004_i2c_data=0;
	//
 	for(i=0;i<8;i++)
 	{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		bs004_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	bs004_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
 	}
	BS004_I2C_SendACK();
 	return bs004_i2c_data;
}

void BS004_I2C_Receive14Bytes(u8 *anbt_i2c_data_buffer)
{
	u8 i,j;
	u8 anbt_i2c_data;

	for(j=0;j<13;j++)
	{
		BS004_I2C_SDA_1;
		BS004_I2C_SCL_0; 
		anbt_i2c_data=0;
		//
		for(i=0;i<8;i++)
		{
  		BS004_I2C_SCL_1;
  		BS004_I2C_NOP; 
  		anbt_i2c_data<<=1;
			//
  		if(BS004_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  		BS004_I2C_SCL_0;  
  		BS004_I2C_NOP;         
		}
		anbt_i2c_data_buffer[j]=anbt_i2c_data;
		BS004_I2C_SendACK();
	}
	//
	BS004_I2C_SDA_1;
	BS004_I2C_SCL_0; 
	anbt_i2c_data=0;
	for(i=0;i<8;i++)
	{
  	BS004_I2C_SCL_1;
  	BS004_I2C_NOP; 
  	anbt_i2c_data<<=1;
			//
  	if(BS004_I2C_SDA_STATE)	anbt_i2c_data|=0x01; 
  
  	BS004_I2C_SCL_0;  
  	BS004_I2C_NOP;         
	}
	anbt_i2c_data_buffer[13]=anbt_i2c_data;
	BS004_I2C_SendNACK();
}
//

void BS004_MPU6050_Init(void)	
{
	unsigned char mpu6050_chip_id;
	//
	BS004_I2C_Configuration();
	//
	BS004_MPU6050_PWM_CFG_FUN(); 				//圆点博士:设置内部时钟
	BS004_MPU6050_EXIT_SLEEP_FUN();    	//圆点博士:退出休眠模式
	BS004_MPU6050_GYRO_CFG_FUN();      	//圆点博士:设置陀螺仪量程
	BS004_MPU6050_ACCEL_CFG_FUN();     	//圆点博士:设置加速度量程
	BS004_MPU6050_USER_CTRL_FUN();
	BS004_MPU6050_I2CBYPASS_CFG_FUN(); 	//圆点博士:设置电磁读写模式
	BS004_MPU6050_INT_CFG_FUN();
	//
	mpu6050_chip_id=BS004_MPU6050_GYRO_WHOAMI_FUN();
	BS004_COM1_Send_Str_Head();
	BS004_COM1_Send_Str_Body("mpu6050 chip id .");		//圆点博士:初始化ADC
	BS004_COM1_Send_4bits_BCD_Num(mpu6050_chip_id);
	BS004_COM1_Send_Str_Tail();	
	//
	BS004_Long_Delay(10000);	//圆点博士:启动延时
}

unsigned char BS004_MPU6050_GYRO_WHOAMI_FUN(void)
{
	unsigned char bs004_mpu6050_gyro_id;
	//
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_WHOAMI_ADDR);  	//圆点博士:发送陀螺仪ID地址
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR+1);      	//圆点博士:发送陀螺仪读地址
	bs004_mpu6050_gyro_id=BS004_I2C_ReceiveByte_NoACK();	//圆点博士:读出陀螺仪ID
	BS004_I2C_STOP();
	//
	return bs004_mpu6050_gyro_id;
	//
}
//
unsigned char BS004_MPU6050_READ_REG_FUN(unsigned char bs004_mpu6050_dev_addr,unsigned char bs004_mpu6050_reg_addr)   
{
	unsigned char bs004_mpu6050_reg;
	
	BS004_I2C_START();
	BS004_I2C_SendByte(bs004_mpu6050_dev_addr);					//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(bs004_mpu6050_reg_addr);    			//圆点博士:发送陀螺仪寄存器地址
	BS004_I2C_START();
	BS004_I2C_SendByte(bs004_mpu6050_dev_addr+1);      	//圆点博士:发送陀螺仪读地址
	bs004_mpu6050_reg=BS004_I2C_ReceiveByte_NoACK();		//圆点博士:读出陀螺仪寄存器值
	BS004_I2C_STOP();
	//
	return bs004_mpu6050_reg;
}

void BS004_MPU6050_PWM_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);				//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_VALUE); //圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}
//
void BS004_MPU6050_GYRO_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);				//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_CFG_ADDR);   	//圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}
//
void BS004_MPU6050_ACCEL_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_ACCEL_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}

void BS004_MPU6050_EXIT_SLEEP_FUN(void)  
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_PWR_MGMT_1_ADDR);  //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_EXIT_SLEEP_VALUE); //圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}

void BS004_MPU6050_USER_CTRL_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);			 //圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_USER_CTRL_ADDR);  //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_USER_CTRL_VALUE); //圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}

void BS004_MPU6050_I2CBYPASS_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);					  //圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_I2CBYPASS_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_I2CBYPASS_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}

void BS004_MPU6050_INT_CFG_FUN(void)   
{
	BS004_I2C_START();
	BS004_I2C_SendByte(BS004_MPU6050_GYRO_ADDR);			//圆点博士:发送陀螺仪写地址
	BS004_I2C_SendByte(BS004_MPU6050_INT_CFG_ADDR);   //圆点博士:发送陀螺仪PWM地址
	BS004_I2C_SendByte(BS004_MPU6050_INT_CFG_VALUE); 	//圆点博士:发送陀螺仪PWM值
	BS004_I2C_STOP();
}

