/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "etootle_pid.h" 
//
extern unsigned int pid_setting_P_value[3];		//圆点博士:PID的P参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_I_value[3];		//圆点博士:PID的I参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_D_value[3];		//圆点博士:PID的D参数，含X轴,Y轴,Z轴
extern unsigned int pid_setting_M_value[3];		//圆点博士:PID的M参数，含X轴,Y轴,Z轴
float bs004_pitch_p,bs004_roll_p,bs004_yaw_p;
float bs004_pitch_i,bs004_roll_i,bs004_yaw_i;
float bs004_pitch_d,bs004_roll_d,bs004_yaw_d;
int bs004_pitch_m,bs004_roll_m,bs004_yaw_m;
float bs004_pitch_mf,bs004_roll_mf,bs004_yaw_mf;
//
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//
extern signed short bs004_fly_gas,bs004_fly_pitch,bs004_fly_roll,bs004_fly_yaw;
float bs004_fly_gas_scale=0,bs004_fly_pitch_scale=0,bs004_fly_roll_scale=0,bs004_fly_yaw_scale=0;
//
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;
float bs004_angle_cur_pitch=0,bs004_angle_cur_roll=0,bs004_angle_cur_yaw=0;
float bs004_angle_last_pitch=0,bs004_angle_last_roll=0,bs004_angle_last_yaw=0;
float bs004_angle_dif_pitch=0,bs004_angle_dif_roll=0,bs004_angle_dif_yaw=0;
float bs004_angle_int_pitch=0,bs004_angle_int_roll=0,bs004_angle_int_yaw=0;
//
float bs004_fly_m1=0,bs004_fly_m2=0,bs004_fly_m3=0,bs004_fly_m4=0;
float bs004_fly_pitch_dir,bs004_fly_roll_dir,bs004_fly_yaw_dir;
//
void BS004_PID_Control(void)
{
	bs004_roll_p =pid_setting_P_value[0]/10.0f;
	bs004_pitch_p=pid_setting_P_value[1]/10.0f;
	bs004_yaw_p  =pid_setting_P_value[2]/10.0f;
	bs004_roll_i =pid_setting_I_value[0]/10000.0f;
	bs004_pitch_i=pid_setting_I_value[1]/10000.0f;
	bs004_yaw_i  =pid_setting_I_value[2]/10000.0f;
	bs004_roll_d =pid_setting_D_value[0]/1000.0f;
	bs004_pitch_d=pid_setting_D_value[1]/1000.0f;
	bs004_yaw_d  =pid_setting_D_value[2]/1000.0f;
	//
	bs004_roll_m=pid_setting_M_value[0];
	bs004_pitch_m=pid_setting_M_value[1];
	bs004_yaw_m=pid_setting_M_value[2];
	//
	if(bs004_roll_m >1000) bs004_roll_m =-(bs004_roll_m-1000);
	if(bs004_pitch_m>1000) bs004_pitch_m=-(bs004_pitch_m-1000);
	if(bs004_yaw_m  >1000) bs004_yaw_m  =-(bs004_yaw_m-1000);
	//
	bs004_roll_mf=bs004_roll_m/10.0f;
	bs004_pitch_mf=bs004_pitch_m/10.0f;
	bs004_yaw_mf=bs004_yaw_m/10.0f;
	//
	bs004_fly_pitch_dir=(float)bs004_fly_pitch/10.0f;
	bs004_fly_roll_dir=(float)bs004_fly_roll/10.0f;
	bs004_fly_yaw_dir=(float)bs004_fly_yaw/10.0f;
	//
	//圆点博士:融合遥控器控制信号
	bs004_fly_m1=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m3=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_pitch*bs004_fly_pitch_scale + bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m2=bs004_fly_gas*bs004_fly_gas_scale - bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	bs004_fly_m4=bs004_fly_gas*bs004_fly_gas_scale + bs004_fly_roll*bs004_fly_roll_scale   - bs004_fly_yaw*bs004_fly_yaw_scale;
	
	bs004_angle_cur_pitch=bs004_imu_pitch-bs004_pitch_mf-bs004_fly_pitch_dir;
	bs004_angle_cur_roll =bs004_imu_roll-bs004_roll_mf-bs004_fly_roll_dir;
	bs004_angle_cur_yaw  =bs004_imu_yaw-bs004_yaw_mf-bs004_fly_yaw_dir;
	//
	//圆点博士:融合P角度比例控制
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_p*bs004_angle_cur_pitch - bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll + bs004_yaw_p*bs004_angle_cur_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_p*bs004_angle_cur_pitch + bs004_roll_p *bs004_angle_cur_roll - bs004_yaw_p*bs004_angle_cur_yaw;
	//
	bs004_angle_int_pitch=bs004_angle_int_pitch+bs004_angle_cur_pitch;
	bs004_angle_int_roll =bs004_angle_int_roll +bs004_angle_cur_roll;	
	bs004_angle_int_roll =bs004_angle_int_yaw +bs004_angle_cur_yaw;	/////////////////////////////////S***********************************/
	//
  //圆点博士:融合I
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_i*bs004_angle_int_pitch - bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll + bs004_yaw_i *bs004_angle_int_yaw;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_i*bs004_angle_int_pitch + bs004_roll_i *bs004_angle_int_roll - bs004_yaw_i *bs004_angle_int_yaw;
	//
	bs004_angle_dif_pitch=bs004_angle_cur_pitch-bs004_angle_last_pitch;
	bs004_angle_dif_roll =bs004_angle_cur_roll-bs004_angle_last_roll;
	bs004_angle_dif_yaw  =bs004_angle_last_yaw-bs004_angle_cur_yaw;
	//
	//圆点博士:融合D
	bs004_fly_m1=bs004_fly_m1 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m2=bs004_fly_m2 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave - bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m3=bs004_fly_m3 - bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave + bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	bs004_fly_m4=bs004_fly_m4 + bs004_pitch_d*bs004_mpu6050_gyro_angel_pitch_ave + bs004_roll_d *bs004_mpu6050_gyro_angel_roll_ave - bs004_yaw_d*bs004_mpu6050_gyro_angel_yaw_ave;
	//
	bs004_angle_last_pitch=bs004_angle_cur_pitch;
	bs004_angle_last_roll =bs004_angle_cur_roll;	
	bs004_angle_last_yaw =bs004_angle_cur_yaw;	
}

