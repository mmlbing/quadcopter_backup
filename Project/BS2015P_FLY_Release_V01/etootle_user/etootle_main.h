/*************************************************************************************************************
圆点博士小四轴飞行器2015+版配套源代码声明:
该源代码仅供参考,圆点博士不对源代码提供任何形式的担保,也不对因使用该源代码而出现的损失负责.
用户可以以学习的目的修改和使用该源代码.
但用户在修改该源代码时,不得移除该部分版权信息，必须保留原版声明.

更多信息，请访问官方网站www.etootle.com, 官方博客:http://weibo.com/xiaosizhou
**************************************************************************************************************/
#include "stm32f10x_lib.h"
#include <math.h>
#include "etootle_led.h"
#include "etootle_bluetooth.h"
#include "etootle_motor.h"
#include "etootle_adc.h"
#include "etootle_sys.h"
#include "etootle_mpu6050.h"
#include "etootle_imu.h"
#include "etootle_parameter.h"

#define IDLE 0
#define UPDATE_COMMAND 1
#define UPDATE_MOTOR   2
#define RESET_MOTOR    3

extern float bs004_mpu6050_gyro_scale,bs004_mpu6050_pi_scale,bs004_gyro_to_rad_scale,bs004_hmc5883l_mag_scale;
extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;
extern signed short  bs004_fly_gas,bs004_fly_pitch,bs004_fly_roll,bs004_fly_yaw;
extern unsigned char BS004_Ctrl_Gas,BS004_Ctrl_Valid,BS004_Ctrl_Gas_Noise;
extern signed char BS004_Ctrl_Pitch,BS004_Ctrl_Roll,BS004_Ctrl_Yaw,BS004_Ctrl_Dir_Noise;
extern float bs004_fly_m1,bs004_fly_m2,bs004_fly_m3,bs004_fly_m4;
extern signed short bs004_fly_m1_out,bs004_fly_m2_out,bs004_fly_m3_out,bs004_fly_m4_out;	

unsigned char BS004_COM1_Task_Process(void);
void BS004_Quad_Calculation(void);
void ANBT_SEND_DMP_EULER_DATA(void);
void BS004_Motor_Control(void);


