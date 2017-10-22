#include "MPU6050.h"
#include <math.h>

short gyro[3], accel[3], sensors;
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
	
	float sens;
	unsigned short accel_sens;
	int result;
	//    char test_packet[4] = {0};
	long gyro[3], accel[3];

	result = mpu_run_self_test(gyro, accel);
	if (result == 0x03)
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		PrintChar("run self check successed...\n");
	}
	else
	{
		PrintChar("run self test failed! but we still set the bias...\n");
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

int MPU6050_Init(void)
{
	char result;
	I2CInit();
	result = mpu_init();
	if(!result)
	{
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
  }
	else
		return 1;
}

void MPU6050_Get_PRY(float *PRY)
{
	unsigned long sensor_timestamp;
	
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
		PRY[0] = asin( -2*q[1]*q[3] + 2*q[0]*q[2] ) *57.3;
		PRY[1] = atan2( 2*q[2]*q[3] + 2*q[0]*q[1], -2*q[1]*q[1] - 2*q[2]*q[2] + 1 ) *57.3;
		PRY[2] = atan2( 2*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3] ) *57.3;
	}
	if(sensors & INV_XYZ_GYRO)
	{
	}
	if(sensors & INV_XYZ_ACCEL)
	{
	}
	UART1_ReportMotion(accel,gyro);
}
