#include <STC15W4K60S4.H>
#include <EEPROM.h>
#include <intrins.h>
#include <NRF24L01.H>
#include <MPU6050.H>
#define CMD_IDLE    0            
#define CMD_READ    1              
#define CMD_PROGRAM 2               
#define CMD_ERASE   3              

#define ENABLE_IAP 0x80           //if SYSCLK<30MHz
extern int g_x,g_y,g_z;            //陀螺仪矫正参数
void IapIdle()
{
    IAP_CONTR = 0;                 
    IAP_CMD = 0;                    
    IAP_TRIG = 0;                  
    IAP_ADDRH = 0x80;              
    IAP_ADDRL = 0;
}
unsigned char IapReadByte(unsigned int addr) //读取1字节
{
    unsigned char dat;                  
    IAP_CONTR = ENABLE_IAP;       
    IAP_CMD = CMD_READ;           
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;          
    IAP_TRIG = 0x5a;             
    IAP_TRIG = 0xa5;             
    _nop_();                      
    dat = IAP_DATA;
    IapIdle();	
    return dat;                   
}

void IapProgramByte(unsigned int addr, unsigned char dat) //写入1字节
{
    IAP_CONTR = ENABLE_IAP;        
    IAP_CMD = CMD_PROGRAM;        
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;         
    IAP_DATA = dat;                
    IAP_TRIG = 0x5a;              
    IAP_TRIG = 0xa5;                
    _nop_();    
    IapIdle();	
}

void IapEraseSector(unsigned int addr) //扇区擦除
{
    IAP_CONTR = ENABLE_IAP;        
    IAP_CMD = CMD_ERASE;          
    IAP_ADDRL = addr;             
    IAP_ADDRH = addr >> 8;         
    IAP_TRIG = 0x5a;              
    IAP_TRIG = 0xa5;               
    _nop_(); 
    IapIdle();	
}
void IAPWrite()   //陀螺仪校准
{
	IapEraseSector(0xC800); //扇区擦除
	IapEraseSector(0xC801);
	IapEraseSector(0xC802);
	IapEraseSector(0xC803);
	IapEraseSector(0xC804);
	IapEraseSector(0xC805);
	IapEraseSector(0xC806);
	g_y = GetData(GYRO_YOUT_H);  //读取陀螺仪数据
	g_x = GetData(GYRO_XOUT_H);
	g_z = GetData(GYRO_ZOUT_H);
	IapProgramByte(0xC800,g_y>>8);  //写入数据至EEPROM
	IapProgramByte(0xC801,g_y); 
	IapProgramByte(0xC802,g_x>>8);
	IapProgramByte(0xC803,g_x);
	IapProgramByte(0xC804,g_z>>8);
	IapProgramByte(0xC805,g_z);
	IapProgramByte(0xC806,1);
}
void IAPRead()
{
	unsigned char i;
	i=IapReadByte(0xC806);
	if(i==1)      //判断数据是否已经更新过，更新过才读取数据
	{
	g_y=IapReadByte(0xC800)<<8|IapReadByte(0xC801);
	g_x=IapReadByte(0xC802)<<8|IapReadByte(0xC803);
	g_z=IapReadByte(0xC804)<<8|IapReadByte(0xC805);
	}
}

