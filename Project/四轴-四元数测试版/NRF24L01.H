#ifndef __NRF24L01_H
#define __NRF24L01_H
typedef unsigned char uchar;
typedef unsigned char uint;	
sbit 	MISO	=P1^1;
sbit 	MOSI	=P1^0;
sbit	SCK	  =P6^3;
sbit	CE	  =P6^1;
sbit	CSN		=P6^2;
sbit	IRQ		=P4^7;
void Delay(unsigned int s);
void inerDelay_us(unsigned char n);
void init_NRF24L01(void);
uint SPI_RW(uint uchar);
uchar SPI_Read(uchar reg);
void SetRX_Mode(void);
uint SPI_RW_Reg(uchar reg, uchar value);
uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars);
uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars);
unsigned char nRF24L01_RxPacket(unsigned char* rx_buf);





#endif  