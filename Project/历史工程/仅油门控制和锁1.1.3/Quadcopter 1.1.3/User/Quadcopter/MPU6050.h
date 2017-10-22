#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "stm32f10x.h"
#include "STM32_I2C.h"

#include <stdio.h>
#include "quad_uart.h"

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

#define q30  1073741824.0f

int MPU6050_Init(void);
void MPU6050_Get_PRY(void);
