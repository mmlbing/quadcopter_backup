#include "MPU6050.h"
#include <math.h>

#define MPU6050_DEBUG

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
	
	float pitchOutput,rollOutput,yawOutput;			//最终输出
}pid;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];
	
		float sens;
		unsigned short accel_sens;

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x03)
		{
			/* Test passed. We can trust the gyro data here, so let's push it down
			 * to the DMP.
			 */
			PrintChar("MPU6050 self test successed...\n");
		}
		else
		{
			/* Test failed.but here we still set the bais....*/
			PrintChar("MPU6050 self test failed!!!but we still set the bias...\n");
		}
		
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		PrintChar("bias set...\n");
}

/**
  * @brief  初始化MPU6050.
	* @detail	首先初始化STM32的IIC接口，然后初始化MPU6050并启用其DMP.
	* @resut	可直接调用MPU6050_Get_PRY()得到欧拉角.
  * @param  None
  * @retval None
  */
int MPU6050_Init(void)
{
	unsigned char result;
	I2CInit();
	I2CReadBuffer(0x69,0x75,1,&result);
	if(result != 0x68)
		PrintChar("who am I check failed!\n");
	result = 0;
	result = mpu_init();
	if(!result)
	{
#ifdef MPU6050_DEBUG
		PrintChar("mpu initialization complete......\n ");
		
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  	 PrintChar("mpu_set_sensor complete ......\n");
	  else
	  	 PrintChar("mpu_set_sensor come across error ......\n");
		
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  	 PrintChar("mpu_configure_fifo complete ......\n");
	  else
	  	 PrintChar("mpu_configure_fifo come across error ......\n");
		
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  	 PrintChar("mpu_set_sample_rate complete ......\n");
	  else
	  	 PrintChar("mpu_set_sample_rate error ......\n");
		
	  if(!dmp_load_motion_driver_firmware())
	  	PrintChar("dmp_load_motion_driver_firmware complete ......\n");
	  else
	  	PrintChar("dmp_load_motion_driver_firmware come across error ......\n");
		
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  	 PrintChar("dmp_set_orientation complete ......\n");
	  else
	  	 PrintChar("dmp_set_orientation come across error ......\n");
		
	  if( !dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
													 DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
													 DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) )
	  	 PrintChar("dmp_enable_feature complete ......\n");
	  else
	  	 PrintChar("dmp_enable_feature come across error ......\n");
		
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  	 PrintChar("dmp_set_fifo_rate complete ......\n");
	  else
	  	 PrintChar("dmp_set_fifo_rate come across error ......\n");
		
	  run_self_test();
	  if(!mpu_set_dmp_state(1))
	  	 PrintChar("mpu_set_dmp_state complete ......\n");
	  else
	  	 PrintChar("mpu_set_dmp_state come across error ......\n");
		return 0;
#else
	  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	  mpu_set_sample_rate(DEFAULT_MPU_HZ);
	  dmp_load_motion_driver_firmware();
		dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
													 DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
													 DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	  run_self_test();
	  mpu_set_dmp_state(1);
		return 0;
#endif
  }
	else
		return 1;
}

/**
  * @brief  得到MPU6050的DMP输出的四元数，并转化为欧拉角.
	* @detail	
	* @resut	可得到欧拉角.
  * @param  要存储的位置，三元数组.
  * @retval None
  */
void MPU6050_Get_PRY(void)
{
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];
	
	float q[4];
	
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 
	if (sensors & INV_WXYZ_QUAT )
	{
		q[0]=quat[0] / q30;
		q[1]=quat[1] / q30;
		q[2]=quat[2] / q30;
		q[3]=quat[3] / q30;
		pid.pryPitch = asin( -2*q[1]*q[3] + 2*q[0]*q[2] ) *57.3;
		pid.pryRoll = atan2( 2*q[2]*q[3] + 2*q[0]*q[1], -2*q[1]*q[1] - 2*q[2]*q[2] + 1 ) *57.3;
		pid.pryYaw = atan2( 2*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3] ) *57.3;
	}
	if(sensors & INV_XYZ_GYRO)
	{
		pid.gyroX = gyro[0];
		pid.gyroY = gyro[1];
		pid.gyroZ = gyro[2];
	}
	if(sensors & INV_XYZ_ACCEL)
	{
	}
	/*if( !( (sensors & INV_WXYZ_QUAT)||(sensors & INV_XYZ_GYRO)||(sensors & INV_XYZ_ACCEL) ) )
	{
		readfailed++;
	}
	else
		readsucce++;*/
}
