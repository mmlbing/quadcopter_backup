#include "stm32f10x.h"

extern struct MOTOR{
	uint16_t motor_1;
	uint16_t motor_2;
	uint16_t motor_3;
	uint16_t motor_4;
}motor;

void QuadPID(void);
