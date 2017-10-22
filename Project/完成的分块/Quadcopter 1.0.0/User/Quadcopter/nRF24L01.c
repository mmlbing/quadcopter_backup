#include "nRF24L01.h"

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址

/************************************************************************
SPI操作相关函数
************************************************************************/
//SPI初始化函数
void SPI1_Init(void)
{	 
	RCC->APB2ENR|=1<<2;       //PORTA时钟使能 	 
	RCC->APB2ENR|=1<<12;      //SPI1时钟使能 
		   
	//这里只针对SPI口初始化
	GPIOA->CRL&=0x000FFFFF; 
	GPIOA->CRL|=0xBBB00000;		//PA5.6.7复用
	GPIOA->ODR|=0x7<<5;    		//PA5.6.7上拉
		
	SPI1->CR1|=0<<10;					//全双工模式	
	SPI1->CR1|=1<<9;					//软件nss管理
	SPI1->CR1|=1<<8;  

	SPI1->CR1|=1<<2; 					//SPI主机
	SPI1->CR1|=0<<11;					//8bit数据格式	
	//对24L01要设置 CPHA=0;CPOL=0;
	SPI1->CR1|=0<<1;						//CPOL=0时空闲模式下SCK为1 
	//SPI1->CR1|=1<<1; 					//空闲模式下SCK为1 CPOL=1			   
	SPI1->CR1|=0<<0;					//第一个时钟的下降沿,CPHA=1 CPOL=1	   
	SPI1->CR1|=7<<3;					//Fsck=Fcpu/256
	SPI1->CR1|=0<<7;					//MSBfirst   
	SPI1->CR1|=1<<6;					//SPI设备使能
	SPI1_ReadWriteByte(0xff);	//启动传输		 
}

//SPI 速度设置函数
//SpeedSet:
//SPI_SPEED_2   2分频   (SPI 36M@sys 72M)
//SPI_SPEED_8   8分频   (SPI 9M@sys 72M)
//SPI_SPEED_16  16分频  (SPI 4.5M@sys 72M)
//SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)
void SPI1_SetSpeed(uint8_t  SpeedSet)
{
	SPI1->CR1&=0xFFC7;//Fsck=Fcpu/256
	if(SpeedSet==SPI_SPEED_2)//二分频
	{
		SPI1->CR1|=0<<3;//Fsck=Fpclk/2=36Mhz	
	}else if(SpeedSet==SPI_SPEED_8)//八分频 
	{
		SPI1->CR1|=2<<3;//Fsck=Fpclk/8=9Mhz	
	}else if(SpeedSet==SPI_SPEED_16)//十六分频
	{
		SPI1->CR1|=3<<3;//Fsck=Fpclk/16=4.5Mhz
	}else			 	 //256分频
	{
		SPI1->CR1|=7<<3; //Fsck=Fpclk/256=281.25Khz 低速模式
	}
	SPI1->CR1|=1<<6; //SPI设备使能	  
}

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t	SPI1_ReadWriteByte(uint8_t  TxData)
{		
	uint8_t  retry=0;				 
	while((SPI1->SR&1<<1)==0)//等待发送区空	
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI1->DR=TxData;	 	  //发送一个byte 
	retry=0;
	while((SPI1->SR&1<<0)==0) //等待接收完一个byte  
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI1->DR;          //返回收到的数据				    
}


/************************************************************************
nRF24L01相关函数
************************************************************************/
void nRF24L01_SPI1_Init(void)
{
	RCC->APB2ENR |= 1<<2;    		//使能PORTA口时钟
	GPIOA->CRL &= 0xFFFFFF00;		//PA0.1输出
	GPIOA->CRL |= 0x00000033; 
	GPIOA->ODR |= 0x00000003;	  //PA0.1 输出1
	
	GPIOA->CRL &= 0xFFFFF0FF;		//PA2输入模式
	
	
	SPI1_Init();    						//初始化SPI
	NRF24L01_CE_LOW;			 			//使能24L01
	NRF24L01_CSN_HIGH;					//SPI片选取消
}

uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
	uint8_t i;
	SPI1_SetSpeed(SPI_SPEED_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)
	{
		if(buf[i]!=0xA5)
			break;
	}		
	if(i!=5)
		return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}

//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	NRF24L01_CSN_LOW;                 	//使能SPI传输
	status =SPI1_ReadWriteByte(reg);		//发送寄存器号 
	SPI1_ReadWriteByte(value);      		//写入寄存器的值
	NRF24L01_CSN_HIGH;                  //禁止SPI传输	   
	return(status);       							//返回状态值
}

//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	NRF24L01_CSN_LOW;// = 0;          //使能SPI传输		
	SPI1_ReadWriteByte(reg);   //发送寄存器号
	reg_val=SPI1_ReadWriteByte(0xFF);//读取寄存器内容
	NRF24L01_CSN_HIGH;// = 1;          //禁止SPI传输		    
	return(reg_val);           //返回状态值
}

//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	    
 	NRF24L01_CSN_LOW;									      //使能SPI传输
	status = SPI1_ReadWriteByte(reg);				//发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		SPI1_ReadWriteByte(*pBuf++); 					//写入数据	 
	NRF24L01_CSN_HIGH;											//关闭SPI传输
	return status;													//返回读到的状态值
}

//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status,u8_ctr;	       
	NRF24L01_CSN_LOW;				            			//使能SPI传输
	status=SPI1_ReadWriteByte(reg);						//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
		pBuf[u8_ctr] = SPI1_ReadWriteByte(0xFF);//读出数据
	NRF24L01_CSN_HIGH;						      			//关闭SPI传输
	return status;      										  //返回读到的状态值
}
		   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
 	SPI1_SetSpeed(SPI_SPEED_8);															//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF24L01_CE_LOW;
  NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	//写数据到TX BUF  32个字节
 	NRF24L01_CE_HIGH;																				//启动发送	   
	while(NRF24L01_IRQ != 0);																	//等待发送完成
	state=NRF24L01_Read_Reg(STATUS);  											//读取状态寄存器的值	   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS, state); 						//清除TX_DS或MAX_RT中断标志
	if(state & MAX_TX)																				//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xFF);										//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(state & TX_OK)																				//发送完成
	{
		return TX_OK;
	}
	return 0xff;																						//其他原因发送失败
}

//启动NRF24L01接收一次数据
//txbuf:存放数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;		    							   
	SPI1_SetSpeed(SPI_SPEED_8); 														//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	state=NRF24L01_Read_Reg(STATUS);  											//读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+STATUS, state); 						//清除TX_DS或MAX_RT中断标志
	if(state & RX_OK)																				//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);										//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;																								//没收到任何数据
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE_LOW;	  
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH);//写RX节点地址

	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);    					//使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01);					//使能通道0的接收地址  	 
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,0);	     					//设置RF通信频率		  
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);					//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG, 0x0f);						//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	NRF24L01_CE_HIGH;																			//CE为高,进入接收模式 
}

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{
	NRF24L01_CE_LOW;	    
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+TX_ADDR,(uint8_t *)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
	NRF24L01_Write_Buf(NRF24L01_WRITE_REG+RX_ADDR_P0,(uint8_t *)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_CH,0);       //设置RF通道为40
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+RF_SETUP,0x07);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(NRF24L01_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE_HIGH;//CE为高,10us后启动发送
}
