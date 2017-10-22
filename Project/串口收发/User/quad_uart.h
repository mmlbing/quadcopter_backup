#include <stdint.h>
#include "stm32f10x_usart.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void Initial_UART1(uint32_t baudrate);
void Uart1_Put_Buf(uint8_t *tx_buffer,uint8_t count);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_ReportIMU(int16_t pitch,int16_t roll,int16_t yaw,int16_t tempr);
void UART1_ReportGyro(int16_t gyroX, int16_t gyroY, int16_t gyroZ);
void UART1_ReportMotion(int16_t accel[3], int16_t gyro[3]);
void PrintChar(char *strings);

void UAR1_ReportMotor(void);
