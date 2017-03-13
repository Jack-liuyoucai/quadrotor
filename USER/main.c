#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	
#include "spi.h"
#include "exti.h"
#include "timer.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "STM32_I2C.h"
#include "math.h"
#include "24L01.h"	
#include "define.h"
#include "imu.h"
#include "control.h"

extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;  
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ HMC;
extern float rool,pitch;
extern int Motor1,Motor2,Motor3,Motor4; 
extern struct DATA_XYZ ACC;
extern float Pitch,Rool,Yaw;

u8 RX_buf[32]={0}; 
u8 TX_buf[32]={0};
u8 delays=10;	 
u8 wireless=0;
u32 loss_wireless=0;
extern u8 FLY_EN;  
int main(void)			
{			  
	Stm32_Clock_Init(9);	//系统时钟设置	 
	delay_init(72);	   		//延时初始化
	delay_ms(1000);			 //硬件做好准备
	RCC->APB2ENR|=1<<0;	  	  // 开辅助时钟	
	AFIO->MAPR = 0x02000000;  // 禁用JTAG	 
	LED_Init();				   //LED初始化
	Show_str_leds();		   //闪烁LED后关掉，检验信号对错
 	NRF24L01_Init();			//  无线初始化
  	while( NRF24L01_Check());	//接受机与发射机密码匹配
 	NRF24L01_RX_Mode();	//拿出数据包
	LED_RED_ON;	 
	delays=20;	     		
    MPU6050_INIT();delay_ms(50);
	MPU6050_INIT();delay_ms(50);
	Get_OFFSET(500); 	  //取500次的平均值，对角度的标定
    delays=10;			  	
	if(ACC_OFFSET.X==0&&ACC_OFFSET.Y==0){ALL_LEDS_ON();while(1);}	 // 传感器失败,开灯阻塞 
	PID_INIT();	  
	TIM2_PWM_Init(999,2);  	 //42us的PWM  百分之百对应值 1000
	TIM3_Int_Init(20,7199); // 定时2MS 	 
	delay_ms(800); 		//延时给硬件做好准备
	ALL_LEDS_OFF();	  //关掉2个LED
	while(1) 
	{	  
		if(wireless==1) LED_twinkle();	   //接收信号正常，LED闪烁
	} 		  		 
}
//////××××××××××2ms的中断××××××××××××××××××////////
float 	angle=0;
long acc_hold[3]={0};
u16 tms=0;
float p_long=0.0,r_long=0.0;
float adj_r=0.0,adj_p=0.0;

void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{					       				   				     	    	 
    	MPU6050_READ();	   //经IIC读取传感器MPU6050的信号
		MPU6050_CONVENT(); //将读取的信号转化成角度与角加速度
		ACC_SMOOTH(10);  // 角度的平滑滤波
		RX_buf[31]= 0;   
		MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);	//函数内转成弧度		
		NRF24L01_RxPacket(RX_buf,TX_buf);	// 读取发射信号			   		
		if(RX_buf[31]==0)		  //  未能正常接收
		{
			loss_wireless++;
		if(	FLY_EN==1&&loss_wireless>50)  //信号丢失 100ms 
			{
				wireless=0;
			
			}
		}
		else if(RX_buf[31]==0XAA)  // 正常接收  
		{
			READ_CONTROL_COMMAND(RX_buf); //转化读取的发射信号	
			LED_GRE_OFF;	
			loss_wireless=0;
			wireless=1;	
		} 	
		STABLE_WITH_PID();	  //将接收信号与传感器信号融合，经PID控制器，控制飞行器飞行      			   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    			    		  			   
}
 
