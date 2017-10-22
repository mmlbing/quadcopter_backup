//四川 绵阳      西南科技大学 信息工程学院 电气13级 刘其民  QQ：1203933924  
//硬件参数：
//电池:1S/3.7V电池 推荐300-500mAh左右
//电机/桨:720空心杯/59MM桨
//MCU IAP15W4K61S4@28.000MHZ  
//陀螺仪加速度计：MPU-6050 
//磁场传感计:HMC5883L
//无线芯片:NRF24L01
//电机驱动MOS管:AO3400
//MOS管保护用肖特基:BAT54ST
//下载口保护用瞬态抑制二极管:RCLAMP0524P
//机架尺寸:94mm*94mm
//标准PID控制公式:PID=P*e(n)+I*[(e(n)+e(n-1)+...+e(0)]+D*[e(n)-e(n-1)]

//数据定义说明：
//data 51单片机片内RAM最前面128字节RAM 用ACC读写，速度最快
//idata 片内RAM最前面256字节的RAM 包括data 用类似指针模式访问 适合用于指针操作
//pdata 外部扩展RAM的前256字节的RAM
//xdata 外部扩展RAM 用DPTR访问
#include <STC15W4K60S4.H>
#include <intrins.h>
#include <NRF24L01.H>
#include <MPU6050.H>
#include <math.h>
#include <STC15W4KPWM.H>
void Time0_Init();   //定时器初始化
void update();       //陀螺仪矫正
void Kalman_Filter(float Accel,float Gyro);   //X轴卡尔曼滤波
void Kalman_Filter_Y(float Accel,float Gyro);	//Y轴卡尔曼滤波	
//*************************************自整定PID相关参数************************************************
//unsigned char xdata ERRORPID[512]={0};
//unsigned char xdata ZSYPID[100]={0};
//unsigned long int xdata ALLERROR[100]={0};
//unsigned long int TE=0,TZA=0,ADD=0,THEEND=0,LESTERROR=0;
//float BESTPID=0;
//******************************************************************************************************
unsigned char PID=0;                         //无线串口PID调整延时用
int ich1=0,ich2=0,ich3=0,ich4=0;				     //无线串口待发送数据
int speed0=0,speed1=0,speed2=0,speed3=0,V=0; //电机控制参数
int PWM0=0,PWM1=0,PWM2=0,PWM3=0;             //电机控制参数
unsigned char TxBuf[20]={0};
unsigned char RxBuf[20]={0};
double g_x=0,g_y=0,g_z=0;                    //陀螺仪静差消除参数
double PID_x=0,PID_y=0,PID_z=0;              //每根轴PID控制量输出
//******角度参数****************************************************************************************
double Angle_ax=0,Angle_ay=0,Angle_az=0;    //由加速度计算的倾斜角度
double Angle_gy=0,Angle_gx=0,Angle_gz=0;    //由角速度计算的倾斜角度
double AngleAx=0,AngleAy=0;                 //反三角函数处理后的角度值
double Angle=0,Angley=0;                    //最终倾斜角度
double Anglelate=0,Anglelatey=0,Anglezlate=0;//存储上一次角度数据
double Ax=0,Ay=0;                            //加入遥控器控制量后的角度
float idata dt=0.008;                              //系统周期
//******卡尔曼滤波参数-X轴******************************************************************************
float data Q_angle=0.001;     //对加速度计的信任度
float data Q_gyro=0.003;      //对陀螺仪的信任度
float data R_angle=0.5;
char  data C_0 = 1;
float data Q_bias, Angle_err;
float data PCt_0, PCt_1, E;
float data K_0, K_1, t_0, t_1;
float data Pdot[4] ={0,0,0,0};
float data PP[2][2] = { { 1, 0 },{ 0, 1 } };
void Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; //陀螺仪积分角度（测量值-陀螺仪偏差)*dt  
//Angle相当于是系统的预测值
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // 先验估计误差协方差的微分
	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=  Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // 先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - Angle;//估计值与预测值之间的偏差
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;
	K_0 = PCt_0 / E;//卡尔曼增益1 用于计算最优估计值
	K_1 = PCt_1 / E;//卡尔曼增益2 用于计算最后估计的偏差
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		 //更新协方差矩阵
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	Angle	+= K_0 * Angle_err;//根据卡尔曼增益1算出最优角度
	Q_bias	+= K_1 * Angle_err;//根据卡尔曼增益2算出预测值偏差
}
//******卡尔曼滤波参数-Y轴***************************************************************************
		
float data Q_angley=0.001;  
float data Q_gyroy=0.003;
float data R_angley=0.5;
char  data C_0y = 1;
float idata Q_biasy, Angle_erry;
float idata PCt_0y, PCt_1y, Ey;
float idata K_0y, K_1y, t_0y, t_1y;
float idata Pdoty[4] ={0,0,0,0};
float idata PPy[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_Y(float Accely,float Gyroy)		
{
	Angley+=(Gyroy - Q_biasy) * dt; 
	Pdoty[0]=Q_angley - PPy[0][1] - PPy[1][0]; 
	Pdoty[1]=- PPy[1][1];
	Pdoty[2]=- PPy[1][1];
	Pdoty[3]=Q_gyroy;
	PPy[0][0] += Pdoty[0] * dt;   
	PPy[0][1] += Pdoty[1] * dt;   
	PPy[1][0] += Pdoty[2] * dt;
	PPy[1][1] += Pdoty[3] * dt;
	Angle_erry = Accely - Angley;	
	PCt_0y = C_0y * PPy[0][0];
	PCt_1y = C_0y * PPy[1][0];
	Ey = R_angley + C_0y * PCt_0y;
	K_0y = PCt_0y / Ey;
	K_1y = PCt_1y / Ey;
	t_0y = PCt_0y;
	t_1y = C_0y * PPy[0][1];
	PPy[0][0] -= K_0y * t_0y;		
	PPy[0][1] -= K_0y * t_1y;
	PPy[1][0] -= K_1y * t_0y;
	PPy[1][1] -= K_1y * t_1y;
	Angley	+= K_0y * Angle_erry;	
	Q_biasy	+= K_1y * Angle_erry;	
}
//************姿态处理和PID**************************************************************************
float idata YP=4.5,YD=115.0,YI=0.05,ERRORX;
float idata XP=4.5,XD=115.0,XI=0.05,ERRORY;
float idata ZP=4.0,ZD=2.0;
void Angle_Calcu(void)
{
	Angle_ax=(GetData(ACCEL_XOUT_H))/8192;  //加速度处理
	Angle_az=(GetData(ACCEL_ZOUT_H))/8192;  //加速度量程 +-4g/S
	Angle_ay=(GetData(ACCEL_YOUT_H))/8192;	//转换关系8192LSB/g

	Angle_gx=(GetData(GYRO_XOUT_H)-g_x)/65.5;   //陀螺仪处理
	Angle_gy=(GetData(GYRO_YOUT_H)-g_y)/65.5;   //陀螺仪量程 +-500度/S
	Angle_gz=(GetData(GYRO_ZOUT_H)-g_z)/65.5;   //转换关系65.5LSB/度
//**************X轴指向*******************************************************************************
	AngleAx=atan(Angle_ax/sqrt(Angle_ay*Angle_ay+Angle_az*Angle_az))*180/3.141592657;
  
	Anglelate=Ax;
	
	//Angle=0.95*(Angle-Angle_gy*dt)+0.05*AngleAx;//一阶互补滤波       这边是-Angle_gy
	
	Kalman_Filter(AngleAx,-Angle_gy);       //卡尔曼滤波
	
	Ax=Angle+((float)RxBuf[1]-128)/7;  
	ERRORX+=3*Ax;                 //加强静差积分强度但是不改变输出控制量的范围
	if(ERRORX>1000){ERRORX=1000;}if(ERRORX<-1000){ERRORX=-1000;}
	PID_y=Ax*XP+ERRORX*XI+(Ax-Anglelate)*XD;	  
	speed0=0+PID_y,speed2=0-PID_y;
//**************Y轴指向*******************************************************************************
  AngleAy=atan(Angle_ay/sqrt(Angle_ax*Angle_ax+Angle_az*Angle_az))*180/3.141592657;

	Anglelatey=Ay;
	
	//Angley=0.95*(Angley+Angle_gx*dt)+0.05*AngleAy;//一阶互补滤波     这边是+Angle_gx
	
	Kalman_Filter_Y(AngleAy,Angle_gx);      //卡尔曼滤波
	
	Ay=Angley+((float)RxBuf[2]-128)/7-RxBuf[4]/20;           //由于结构问题，油门越大，Y轴加速度计输出偏差越大，RxBuf[4]补偿之
	ERRORY+=3*Ay;                 //加强静差积分强度但是不改变输出控制量的范围
	if(ERRORY>1000){ERRORY=1000;}if(ERRORY<-1000){ERRORY=-1000;}
	PID_x=Ay*YP+ERRORY*YI+(Ay-Anglelatey)*YD;	  
	speed3=0+PID_x,speed1=0-PID_x;	   
//**************Z轴指向*******************************************************************************
	Angle_gz+=(RxBuf[3]-128)/10;
	PID_z=(Angle_gz)*ZP+(Angle_gz-Anglezlate)*ZD;
	Anglezlate=Angle_gz;
	speed0=speed0+PID_z,speed2=speed2+PID_z;
	speed1=speed1-PID_z,speed3=speed3-PID_z;
//*****************无线串口相关***********************************************************************
	ich1=Ax;
	ich2=Ay;
	ich3=AngleAy;
	ich4=XD;
//**************速度更新******************************************************************************	
	if((1000-RxBuf[4]*4+speed0)>1000)PWM0=1000;
	else if((1000-RxBuf[4]*4+speed0)<0)PWM0=0; 
	else PWM0=(1000-RxBuf[4]*4+speed0);

	if((1000-RxBuf[4]*4+speed1)>1000)PWM1=1000;
	else if((1000-RxBuf[4]*4+speed1)<0)PWM1=0;
	else PWM1=(1000-RxBuf[4]*4+speed1);

	if((1000-RxBuf[4]*4+speed2)>1000)PWM2=1000;
	else if((1000-RxBuf[4]*4+speed2)<0)PWM2=0;
	else PWM2=(1000-RxBuf[4]*4+speed2);

	if((1000-RxBuf[4]*4+speed3)>1000)PWM3=1000;
	else if((1000-RxBuf[4]*4+speed3)<0)PWM3=0;
	else PWM3=(1000-RxBuf[4]*4+speed3);
  if(RxBuf[4]>=10)
	{PWM(PWM1,PWM2,PWM0,PWM3);}//1203
	else
	{PWM(1000,1000,1000,1000);}
//******************************************以下注释内容为PID中D的自动整定模式********************************************
	/*          
	if(RxBuf[4]>60&&THEEND==0)
	{
		if(Ax<0&&Ay>0)
		{
		ERRORPID[TE]=-Ax+Ay;
		}
		else if(Ax>0&&Ay<0)
		{
		ERRORPID[TE]=Ax-Ay;
		}	
		else if(Ax<0&&Ay<0)
		{
		ERRORPID[TE]=-Ax-Ay;
		}	
		else
		{
		ERRORPID[TE]=Ax+Ay;
		}
		TE++;
		if(TE==511)
		{
			TE=0;
			ZSYPID[TZA]=XD;
			XD++;YD++;
			for(ADD=0;ADD<=510;ADD++)
				{
			     ALLERROR[TZA]+=ERRORPID[ADD];
				}
			TZA++;
			if(TZA==100)
			{
				THEEND=1;
				LESTERROR=ALLERROR[1];
				for(TE=1;TE<=99;TE++)
				{
					if(LESTERROR>=ALLERROR[TE])
					{
						LESTERROR=ALLERROR[TE];
						BESTPID=ZSYPID[TE];
					}
				}
				XD=YD=BESTPID;
			}
		}
	}	
	*/
} 
void main()
{
Delay(1000);
PWMGO();
init_NRF24L01();
InitMPU6050();
Time0_Init();
RxBuf[1]=128;
RxBuf[2]=128;
RxBuf[3]=128;
RxBuf[4]=0;
	//SetRX_Mode();
while(1)
{	   
		    //以下为无线串口助手发送程序
				TxBuf[1]=13; TxBuf[6]=13; TxBuf[11]=15; TxBuf[16]=15;
			  if(ich1<0)	 {TxBuf[1]=12; ich1=-ich1;}
				if(ich2<0)	 {TxBuf[6]=12; ich2=-ich2;}
				if(ich3<0)	 {TxBuf[11]=12; ich3=-ich3;}
				if(ich4<0)	 {TxBuf[16]=12; ich4=-ich4;}
				TxBuf[2]=ich1/1000;	TxBuf[3]=ich1/100%10; TxBuf[4]=ich1%100/10;	TxBuf[5]=ich1%10;  	
				TxBuf[7]=ich2/1000;	TxBuf[8]=ich2/100%10; TxBuf[9]=ich2%100/10;	TxBuf[10]=ich2%10;
				TxBuf[12]=ich3/1000; TxBuf[13]=ich3/100%10;	TxBuf[14]=ich3%100/10; TxBuf[15]=ich3%10;
				TxBuf[17]=ich4/1000; TxBuf[18]=ich4/100%10;	TxBuf[19]=ich4%100/10; TxBuf[0]=ich4%10;
				//以上为无线串口助手发送程序
				nRF24L01_TxPacket(TxBuf);
				Delay(1000);
				SetRX_Mode();
				Delay(100);
				nRF24L01_RxPacket(RxBuf);
				if(RxBuf[5]==1)	{update();}
				if(RxBuf[7]==1){PID++;if(PID==10){ZD--;PID=0;}}
				if(RxBuf[8]==1){PID++;if(PID==10){ZD++;PID=0;}}
				if(RxBuf[9]==1){PID++;if(PID==10){ZP-=0.1;PID=0;}}
				if(RxBuf[10]==1){PID++;if(PID==10){ZP+=0.1;PID=0;}}
}
}
void update()   //陀螺仪校准
{
	g_y = GetData(GYRO_YOUT_H);
	g_x = GetData(GYRO_XOUT_H);
	g_z = GetData(GYRO_ZOUT_H);
}
void Time0_Init()		//8ms@28MHz 定时器0 16位12T不自动重载 自动重载偶尔会大姨妈
{
  AUXR &= 0x7F;
	TMOD &= 0xF0;
	TMOD |= 0x01;
	IE  = 0x82;
	TL0 = 0x15;
	TH0 = 0xB7;
	TF0 = 0;
	TR0 = 1;
}
void Time0_Int() interrupt 1     //8毫秒一个周期（125赫兹）
{
	TL0 = 0x15;		
	TH0 = 0xB7;	
  Angle_Calcu();                  //倾角计算  
} 