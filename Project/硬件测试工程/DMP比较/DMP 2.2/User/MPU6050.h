#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "STM32_I2C.h"
#include "quad_uart.h"

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define q30  1073741824.0f

int MPU6050_Init(void);
void MPU6050_Get_PRY(float *PRY);
