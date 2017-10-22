#include "nRF24L01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址

void nRF24L01_XC1_Timer4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
	TIM_OCInitTypeDef  TIM_OCInitStructure;  
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  
 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;													//TIMER时钟预分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 3;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;
	TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

void nRF24L01_IRQ_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//SPI初始化函数
void SPI2_Init(void)
{
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能 	 
	RCC->APB1ENR|=1<<14;      //SPI2时钟使能 
		   
	//这里只针对SPI口初始化
	GPIOB->CRH &= 0x000FFFFF; 
	GPIOB->CRH |= 0xBBB00000;		//PB13.14.15复用 	    
	GPIOB->ODR |= 0x70000000;   //PB13.14.15上拉
		
	SPI2->CR1|=0<<10;					//全双工模式	
	SPI2->CR1|=1<<9;					//软件nss管理
	SPI2->CR1|=1<<8;  

	SPI2->CR1|=1<<2; 					//SPI主机
	SPI2->CR1|=0<<11;					//8bit数据格式	
	//对24L01要设置 CPHA=0;CPOL=0;
	SPI2->CR1|=0<<1;						//CPOL=0时空闲模式下SCK为1 
	//SPI2->CR1|=1<<1; 					//空闲模式下SCK为1 CPOL=1			   
	SPI2->CR1|=0<<0;					//第一个时钟的下降沿,CPHA=1 CPOL=1	   
	SPI2->CR1|=7<<3;					//Fsck=Fcpu/256
	SPI2->CR1|=0<<7;					//MSBfirst   
	SPI2->CR1|=1<<6;					//SPI设备使能
	SPI2_ReadWriteByte(0xff);	//启动传输		 
}

//SPI 速度设置函数
//SpeedSet:
//SPI_SPEED_2   2分频   (SPI 36M@sys 72M)
//SPI_SPEED_8   8分频   (SPI 9M@sys 72M)
//SPI_SPEED_16  16分频  (SPI 4.5M@sys 72M)
//SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)
void SPI2_SetSpeed(uint8_t  SpeedSet)
{
	SPI2->CR1&=0xFFC7;//Fsck=Fcpu/256
	if(SpeedSet==SPI_SPEED_2)//二分频
	{
		SPI2->CR1|=0<<3;//Fsck=Fpclk/2=36Mhz	
	}else if(SpeedSet==SPI_SPEED_8)//八分频 
	{
		SPI2->CR1|=2<<3;//Fsck=Fpclk/8=9Mhz	
	}else if(SpeedSet==SPI_SPEED_16)//十六分频
	{
		SPI2->CR1|=3<<3;//Fsck=Fpclk/16=4.5Mhz
	}else			 	 //256分频
	{
		SPI2->CR1|=7<<3; //Fsck=Fpclk/256=281.25Khz 低速模式
	}
	SPI2->CR1|=1<<6; //SPI设备使能	  
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t	SPI2_ReadWriteByte(uint8_t  TxData)
{
	//while(SPI2->SR & 0x01)
	//	SPI2->DR;
	
	//if(SPI2->SR & 0x02)//等待发送区空			  
		SPI2->DR=TxData;	 	  //发送一个byte 
	
	while((SPI2->SR & 0x80));
	//while(SPI2->SR & 0x01) //等待接收完一个byte    						    
		return SPI2->DR;          //返回收到的数据
	//return 0;
}


/************************************************************************
nRF24L01相关函数
************************************************************************/
void nRF24L01_SPI2_Init(void)
{
	RCC->APB2ENR |= 1<<3;				//使能PORTB口时钟
	RCC->APB2ENR |= 1<<2;    		//使能PORTA口时钟
	GPIOA->CRH &= 0xFFFFF0FF;		//PA10输出
	GPIOB->CRH &= 0xFFF0FFFF;		//PB12输出
	GPIOA->CRH |= 0x00000300;
	GPIOB->CRH |= 0x00030000; 
	GPIOA->ODR |= 1<<10;	  //PA10输出1
	GPIOB->ODR |= 1<<12;	  //PB12输出1
	
	GPIOA->CRH &= 0xFFFF0FFF;		//PA11输入模式/中断脚
	
	nRF24L01_XC1_Timer4_Init();
	nRF24L01_IRQ_Init();

	SPI2_Init();    						//初始化SPI
	nRF24L01_CE_LOW;			 			//使能24L01
	nRF24L01_CSN_HIGH;					//SPI片选取消
}

uint8_t nRF24L01_Check(void)
{
	uint8_t buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	uint8_t i;
	SPI2_SetSpeed(SPI_SPEED_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	nRF24L01_Write_Buf(nRF24L01_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	nRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0xA5)
			break;
	}		
	if(i!=5)
		return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}

//读取状态寄存器
uint8_t nRF24L01_Get_State(void)
{
	uint8_t reg_val;
 	nRF24L01_CSN_LOW;
	reg_val=SPI2_ReadWriteByte(0xFF);
	nRF24L01_CSN_HIGH; 
	return(reg_val);
}

//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t nRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	nRF24L01_CSN_LOW;                 	//使能SPI传输
	status =SPI2_ReadWriteByte(reg);		//发送寄存器号 
	SPI2_ReadWriteByte(value);      		//写入寄存器的值
	nRF24L01_CSN_HIGH;                  //禁止SPI传输	   
	return(status);       							//返回状态值
}

//读取SPI寄存器值
//reg:要读的寄存器
uint8_t nRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	nRF24L01_CSN_LOW;// = 0;          //使能SPI传输		
	SPI2_ReadWriteByte(reg);   //发送寄存器号
	reg_val=SPI2_ReadWriteByte(0xFF);//读取寄存器内容
	nRF24L01_CSN_HIGH;// = 1;          //禁止SPI传输		    
	return(reg_val);           //返回状态值
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t nRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	    
 	nRF24L01_CSN_LOW;									      //使能SPI传输
	status = SPI2_ReadWriteByte(reg);				//发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		SPI2_ReadWriteByte(*pBuf++); 					//写入数据	 
	nRF24L01_CSN_HIGH;											//关闭SPI传输
	return status;													//返回读到的状态值
}

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t nRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	       
	nRF24L01_CSN_LOW;				            			//使能SPI传输
	status=SPI2_ReadWriteByte(reg);						//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
		pBuf[u8_ctr] = SPI2_ReadWriteByte(0xFF);//读出数据
	nRF24L01_CSN_HIGH;						      			//关闭SPI传输
	return status;      										  //返回读到的状态值
}
		   
//启动nRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t nRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;   
	nRF24L01_CE_LOW;
  nRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	//写数据到TX BUF  32个字节
 	nRF24L01_CE_HIGH;																				//启动发送	   
	while(nRF24L01_IRQ != 0);																	//等待发送完成
	nRF24L01_CSN_LOW;
	state=SPI2_ReadWriteByte(0xFF);
	nRF24L01_CSN_HIGH;  
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS, state); 						//清除TX_DS或MAX_RT中断标志
	if(state & MAX_TX)																				//达到最大重发次数
	{
		nRF24L01_Write_Reg(FLUSH_TX,0xFF);										//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(state & TX_OK)																				//发送完成
	{
		return TX_OK;
	}
	return 0xff;																						//其他原因发送失败
}

//启动nRF24L01接收一次数据
//txbuf:存放数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t nRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	nRF24L01_CSN_LOW;
	state=SPI2_ReadWriteByte(0xFF);
	nRF24L01_CSN_HIGH;
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+STATUS, state); 	//清除TX_DS或MAX_RT中断标志
	if(state & RX_OK)																				//接收到数据
	{
		nRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);//读取数据
		nRF24L01_Write_Reg(FLUSH_RX,0xff);										//清除RX FIFO寄存器 
		return 0; 
	}
	return 1;																								//没收到任何数据
}

//该函数初始化nRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void nRF24L01_RX_Mode(void)
{
	nRF24L01_CE_LOW;	  
	nRF24L01_Write_Buf(nRF24L01_WRITE_REG+RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);//写RX节点地址

	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+EN_AA,0x00);    					//使能通道0的自动应答    
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+EN_RXADDR,0x01);					//使能通道0的接收地址  	 
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+RF_CH,40);	     					//设置RF通信频率		  
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+RF_SETUP,0x07);					//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+CONFIG, 0x3f);						//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	nRF24L01_CE_HIGH;																			//CE为高,进入接收模式 
	//nRF24L01_IRQ_Init();
	//delay_ms(1);
}

//该函数初始化nRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void nRF24L01_TX_Mode(void)
{
	nRF24L01_CE_LOW;	    
	nRF24L01_Write_Buf(nRF24L01_WRITE_REG+TX_ADDR,(uint8_t *)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
	nRF24L01_Write_Buf(nRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t *)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+RF_CH,40);       //设置RF通道为40
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+RF_SETUP,0x07);  //设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
	nRF24L01_Write_Reg(nRF24L01_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	nRF24L01_CE_HIGH;//CE为高,10us后启动发送
}
