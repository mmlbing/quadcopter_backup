/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_main.h"
//
unsigned int system_idle_counter=0,system_timer_1ms_event=0,system_timer_state=0,system_status=0;
unsigned int system_timer_counter=0,system_led_timer_counter=0;
extern unsigned char bs004_com_command_ops;
extern unsigned char com_status_is_idle;		
extern unsigned char BS004_Ctrl_Gas,BS004_Ctrl_Valid;
extern signed char   BS004_Ctrl_Pitch,BS004_Ctrl_Roll,BS004_Ctrl_Yaw;
extern unsigned int Motor_BS004_M4,Motor_BS004_M2,Motor_BS004_M3,Motor_BS004_M1,BS004_Motor_Scale;
//
int main()
{
	BS004_RCC_Configuration();				//圆点博士:时钟设置
	BS004_NVIC_Configuration();     	//圆点博士:中断设置
	//
	BS004_SYS_LED_Configuration();    //圆点博士:LED设置
	BS004_SYS_Timer_Configuration();  //圆点博士:系统时间设置
	BS004_LED_GPIO_Configuration();   //圆点博士:LED设置
	//
	BS004_COM1_GPIO_Configuration();	//圆点博士:串口设置
	BS004_COM1_Port_Configuration();  //圆点博士:串口设置
	BS004_ADC_Configuration();        //圆点博士:电压检测设置
	//
	BS004_MPU6050_Init();             //圆点博士:MPU6050初始化
	BS004_Load_Fly_Parameter();			  //圆点博士:装载参数
	BS004_Show_Calibrated_Data();     //圆点博士:检查校验后的MPU6050数据
	//
	BS004_Motor_GPIO_Configuration(); //圆点博士:电机设置
	BS004_Motor_PWM_Configuration(); 	//圆点博士:电机设置
	//
	while(1)
	{
		if(system_timer_1ms_event)			//圆点博士:1MS事件触发
		{
			system_timer_1ms_event=0;			//圆点博士:清除事件
			system_timer_counter++;       //圆点博士:系统计数器
			system_led_timer_counter++;   //圆点博士:LED灯闪烁计数器
			//
			switch (system_timer_state)
			{
				case IDLE:									//圆点博士:空闲状态
					system_idle_counter++;
					BS004_Get_MPU6050_Data();
					//
					if(system_idle_counter>5)
						system_timer_state=UPDATE_COMMAND;
					break;

				case UPDATE_COMMAND:				//圆点博士:读取MPU6050数据和串口数据
					BS004_Get_MPU6050_Data();
					system_status=BS004_COM1_Task_Process();
					//
					if(system_status)
						system_timer_state=UPDATE_MOTOR;
					else
						system_timer_state=RESET_MOTOR;
					break;
				
				case UPDATE_MOTOR:					//圆点博士:更新电机输出
					BS004_Get_MPU6050_Data();
					BS004_Quad_Calculation();
				  BS004_PID_Control();
					BS004_Motor_Control();
					//
					system_timer_state=UPDATE_COMMAND;
					break;

				case RESET_MOTOR:          //圆点博士:复位电机输出
					BS004_Get_MPU6050_Data();
				  BS004_Quad_Calculation();
					BS004_Motor_Reset();
					//
					system_timer_state=UPDATE_COMMAND;
					break;
				
				default:
					break;
			}
		}
		else
		{			
			BS004_SYS_LED_TWINKLE();   //圆点博士:LED灯闪烁
		}
	}	
}
//
void BS004_Quad_Calculation(void)
{
	float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
	//
	bs004_gyro_to_rad_scale=bs004_mpu6050_pi_scale*bs004_mpu6050_gyro_scale;
	//	
	gx=bs004_mpu6050_gyro_angel_pitch_ave/bs004_gyro_to_rad_scale;
	gy=bs004_mpu6050_gyro_angel_roll_ave/bs004_gyro_to_rad_scale;
	gz=bs004_mpu6050_gyro_angel_yaw_ave/bs004_gyro_to_rad_scale;		
	//
	ax=bs004_mpu6050_acc_angel_roll_ave;
	ay=bs004_mpu6050_acc_angel_pitch_ave;	
	az=bs004_mpu6050_acc_angel_yaw_ave;		
	//
	BS004_IMU_Update(ax,ay,az,gx,gy,gz);
}
//
unsigned char BS004_COM1_Task_Process(void)
{
	if(bs004_com_command_ops>0xA0)
	{
		com_status_is_idle=0;					//圆点博士:设置串口忙标志
		BS004_COM1_Communication();   //圆点博士:执行命令
		bs004_com_command_ops=0;			//圆点博士:清空命令操作标志
		com_status_is_idle=1;					//圆点博士:清除串口忙标志
	} 
	else if((bs004_com_command_ops==0xA0))//  respond to gas
	{
		if(BS004_Ctrl_Valid==1)					//圆点博士:检查控制数据是否有效
		{
			com_status_is_idle=0;					//圆点博士:设置串口忙标志			
			//			
			if(BS004_Ctrl_Gas>BS004_Ctrl_Gas_Noise)
				bs004_fly_gas=BS004_Ctrl_Gas-BS004_Ctrl_Gas_Noise;
			else
				bs004_fly_gas=0;
			if(fabs(BS004_Ctrl_Pitch)>BS004_Ctrl_Dir_Noise)
				bs004_fly_pitch=BS004_Ctrl_Pitch;
			else
				bs004_fly_pitch=0;
			if(fabs(BS004_Ctrl_Roll)>BS004_Ctrl_Dir_Noise)
				bs004_fly_roll=BS004_Ctrl_Roll;
			else
				bs004_fly_roll=0;
			if(fabs(BS004_Ctrl_Yaw)>BS004_Ctrl_Dir_Noise)
				bs004_fly_yaw=BS004_Ctrl_Yaw;
			//else
			bs004_fly_yaw=0;//////////////////////////S*********************************************/
			//
			BS004_Ctrl_Valid=0;					//圆点博士:清除控制信号标志
			com_status_is_idle=1;				//圆点博士:清除串口忙标志
		}
	}	
	return bs004_fly_gas;
}
//
void BS004_Motor_Control(void)
{
	bs004_fly_m1_out=BS004_Motor_Speed_Scale(bs004_fly_m1);
	bs004_fly_m2_out=BS004_Motor_Speed_Scale(bs004_fly_m2);	
	bs004_fly_m3_out=BS004_Motor_Speed_Scale(bs004_fly_m3);
	bs004_fly_m4_out=BS004_Motor_Speed_Scale(bs004_fly_m4);	
	//
	TIM_SetCompare4(TIM8,bs004_fly_m1_out);		//圆点博士:更新PWM占空比  
	TIM_SetCompare2(TIM8,bs004_fly_m2_out);	  //圆点博士:更新PWM占空比  
	TIM_SetCompare1(TIM8,bs004_fly_m3_out);   //圆点博士:更新PWM占空比      
	TIM_SetCompare3(TIM8,bs004_fly_m4_out);		//圆点博士:更新PWM占空比 
}
//
void ANBT_SEND_DMP_EULER_DATA(void)
{
	float bs004_display_pitch=0,bs004_display_roll=0,bs004_display_yaw=0;
	unsigned char data_type,checksum=0,euler_data_sign=0,i=0;
	unsigned int bs004_mpu6050_euler_data[3];
	unsigned char bs004_mpu6050_euler_data_buffer[6];
	//
	bs004_display_pitch=bs004_imu_pitch*100;
	bs004_display_roll =bs004_imu_roll*100;
	bs004_display_yaw  =bs004_imu_yaw*100;
	//	
	if(bs004_display_pitch<0) 
	{
		euler_data_sign|=4;
		bs004_display_pitch+=18000;
	}
	if(bs004_display_roll<0) 
	{
		euler_data_sign|=2;	
		bs004_display_roll+=18000;
	}
	if(bs004_display_yaw<0) 
	{
		euler_data_sign|=1;		
		bs004_display_yaw+=18000;
	}
	//
	bs004_mpu6050_euler_data[0]=(unsigned int)bs004_display_pitch;
	bs004_mpu6050_euler_data[1]=(unsigned int)bs004_display_roll;	
	bs004_mpu6050_euler_data[2]=(unsigned int)bs004_display_yaw;	
	//
  for(i=0;i<3;i++) 
	{
		bs004_mpu6050_euler_data_buffer[i*2]=(bs004_mpu6050_euler_data[i]>>8)&0xff;
		bs004_mpu6050_euler_data_buffer[i*2+1]=bs004_mpu6050_euler_data[i]&0xff;
	}
	//
	data_type=0xB0| euler_data_sign;
	checksum=data_type;
	for(i=0;i<6;i++) checksum+=bs004_mpu6050_euler_data_buffer[i];
	checksum&=0xff;
	checksum=~checksum;
	checksum++;
	//
	if(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==SET)
	{
		BS004_COM1_Send_Char(':');
		BS004_COM1_Send_Num(data_type);
		for(i=0;i<6;i++) BS004_COM1_Send_Num(bs004_mpu6050_euler_data_buffer[i]);			
		BS004_COM1_Send_Num(checksum);
		BS004_COM1_Send_Char('/');
		BS004_COM1_Send_Char('\n');
	}
}



