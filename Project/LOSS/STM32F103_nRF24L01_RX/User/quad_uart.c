/***********************************
---------Ӳ���ϵ���������:----------
�ӿڣ�
TXD  -->  PB6  (UART1-TXD)
RXD  -->  PB7 (UART1-RXD)
------------------------------------
***********************************/

#include "quad_uart.h"
#include <string.h>

/**********************
       �ڲ�����
**********************/
/*uint8_t BYTE0(uint16_t data)
{
	return (uint8_t)(data&0x00FF);
}

uint8_t BYTE1(uint16_t data)
{
	return (uint8_t)(data>>8);
}*/


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_UART1(u32 baudrate)
*��������:		��ʼ��UART
���������
		u32 baudrate   ����RS232���ڵĲ�����
���������û��	
*******************************************************************************/
void Initial_UART1(uint32_t baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	
	/* ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
	/* ����UART1 �ķ�������
	����PA9 Ϊ�������  ˢ��Ƶ��50MHz
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* 
	����UART1 �Ľ�������
	����PA10Ϊ�������� 
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 ,ENABLE);
	/* 
	UART1������:
	1.������Ϊ���ó���ָ�������� baudrate;
	2. 8λ����			  USART_WordLength_8b;
	3.һ��ֹͣλ			  USART_StopBits_1;
	4. ����żЧ��			  USART_Parity_No ;
	5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
	6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
	*/
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Ӧ�����õ�UART1
	USART_Init(USART1, &USART_InitStructure);
	//64M-57600-0x457
	USART1->BRR = 0x457;
	//����UART1
	USART_Cmd(USART1, ENABLE);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & USART_FLAG_TXE));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Uart1_Put_Buf(uint8_t *tx_buffer,uint8_t count)
*��������:		UART���ͻ�����tx_buffer�е�����
���������
		*tx_buffer   		��������ʼ��ַ
		count						�����ֽڳ���
���������û��	
*******************************************************************************/
void Uart1_Put_Buf(uint8_t *tx_buffer,uint8_t count)
{
	uint8_t i;
	for(i=0;i<count;i++)
	{
		 UART1_Put_Char(tx_buffer[i]);
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t tempr)
*��������:		����λ�����;�����������̬����
���������
		int16_t yaw ���������ĺ���Ƕȡ���λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
		int16_t pitch ����õ��ĸ����Ƕȣ���λ 0.1�ȡ�-900 - 900 ��Ӧ -90.0 -> 90.0 ��
		int16_t roll  �����õ��ĺ���Ƕȣ���λ0.1�ȡ� -1800 -> 1800 ��Ӧ -180.0  ->  180.0��
		int16_t tempr �¶� �� ��λ0.1���϶�

���������û��	
*******************************************************************************/
void UART1_ReportIMU(int16_t pitch,int16_t roll,int16_t yaw,int16_t tempr)
{
 	uint8_t tx_buffer[16],count = 0,sum = 0,
					i;
	uint16_t temp;
	
	tx_buffer[count++]=0xAA;
	tx_buffer[count++]=0xAA;
	tx_buffer[count++]=0x01;
	tx_buffer[count++]=0;
	
	temp = (int)(roll*100);
	tx_buffer[count++]=BYTE1(temp);
	tx_buffer[count++]=BYTE0(temp);
	
	temp = (int)(pitch*100);
	tx_buffer[count++]=BYTE1(temp);
	tx_buffer[count++]=BYTE0(temp);
	
	temp = (int)(yaw*100);
	tx_buffer[count++]=BYTE1(temp);
	tx_buffer[count++]=BYTE0(temp);
	
	temp = (int)(tempr*100);
	tx_buffer[count++]=BYTE1(temp);
	tx_buffer[count++]=BYTE0(temp);
	
	temp = (int)(tempr*100);
	tx_buffer[count++]=BYTE1(temp);
	tx_buffer[count++]=BYTE0(temp);
		
	tx_buffer[count++]=0xA1;					//����״̬
	
	tx_buffer[3] = count-4;
	
	for(i=0;i<count;i++)
		sum += tx_buffer[i];
		
	tx_buffer[count++]=sum;

	Uart1_Put_Buf(tx_buffer,count);

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
*��������:		����λ�����͵�ǰ�����������ֵ
���������
	int16_t ax  ���ٶ� X��ADC��� ��Χ ��һ���з�������
	int16_t ay  ���ٶ� Y��ADC��� ��Χ ��һ���з�������
	int16_t az  ���ٶ� Z��ADC��� ��Χ ��һ���з�������
	int16_t gx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t gy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t gz  ������ Z��ADC��� ��Χ ��һ���з�������
	int16_t hx  ������ X��ADC��� ��Χ ��һ���з�������
	int16_t hy  ������ Y��ADC��� ��Χ ��һ���з�������
	int16_t hz  ������ Z��ADC��� ��Χ ��һ���з�������
	
���������û��	
*******************************************************************************/
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+8);
	UART1_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}

void PrintChar(char *strings)
{
	uint8_t len,i;
	len = strlen(strings);
	for(i=0;i<len;i++)
		{
			UART1_Put_Char(strings[i]);
		}
}

//------------------------------------------------------
/*void USART1_IRQHandler(void)
{
  
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    // Write one byte to the transmit data register 
//S    USART_SendData(USART1, TxBuffer[TxCounter++]);                    

    // Clear the USART1 transmit interrupt 
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 

//S    if(TxCounter == count)
    {
      // Disable the USART1 Transmit interrupt 
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
  }

}*/


//------------------End of File----------------------------