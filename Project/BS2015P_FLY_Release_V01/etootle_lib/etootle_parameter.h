/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "stm32f10x_it.h"
#include "etootle_bluetooth.h"
#include "etootle_sys.h"
//
extern float bs004_filter_high;
extern float bs004_filter_low;
extern float bs004_filter_time;
extern unsigned int bs004_sys_timer_period;
extern float bs004_mpu6050_gyro_scale;
extern float bs004_mpu6050_pi_scale;
extern float bs004_hmc5883l_mag_scale;
extern unsigned int bs004_motor_pwm_period;
extern unsigned int BS004_Motor_Scale;
extern float bs004_quad_halfT;
extern float bs004_quad_Kp;
extern float bs004_quad_Ki;
//
extern unsigned int pid_setting_P_value[3],pid_setting_I_value[3],pid_setting_D_value[3],pid_setting_M_value[3];
extern float bs004_fly_gas_scale,bs004_fly_pitch_scale,bs004_fly_roll_scale,bs004_fly_yaw_scale;
//
extern signed short int bs004_mpu6050_gyro_pitch_cal,bs004_mpu6050_gyro_roll_cal,bs004_mpu6050_gyro_yaw_cal;
extern signed short int bs004_mpu6050_acc_pitch_cal,bs004_mpu6050_acc_roll_cal,bs004_mpu6050_acc_yaw_cal;
extern signed short int bs004_hmc5883l_mag_pitch_cal,bs004_hmc5883l_mag_roll_cal,bs004_hmc5883l_mag_yaw_cal;
//
void BS004_Load_Filter_Parameter(void);
void BS004_Load_Pid_Parameter(void);
void BS004_Load_Calibration_Parameter(void);
void BS004_Load_Fly_Parameter(void);
//
void BS004_Cal_MPU6050_Data(int *bs004_cal_data);
void BS004_Show_Calibrated_Data(void);




