#include "timer.h"
#include "led.h"

//定时器3中断服务程序	 

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM3->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM3_IRQChannel,2);//抢占1，子优先级3，组2									 
}
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u16 arr,u16 psc)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB1ENR|=1; 		//TIM2时钟使能    
	RCC->APB2ENR|=1<<2;    	//使能PORTA时钟	
	GPIOA->CRL&=0XFFFF0000;	//PA输出
	GPIOA->CRL|=0X0000BBBB;	//复用功能输出 	  	 
	   
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XFFFFFCFF; //清除MAPR的[11:10]

	TIM2->ARR=arr;			//设定计数器自动重装值 
	TIM2->PSC=psc;			//预分频器不分频
	
	TIM2->CCMR1|=6<<4;  	//CH2 PWM2模式
	TIM2->CCMR1|=1<<3;      //CH2预装载使能	 
		
	TIM2->CCMR1|=6<<12;  	//CH2 PWM2模式		 
	TIM2->CCMR1|=1<<11; 	//CH2预装载使能

	TIM2->CCMR2|=6<<4;  	//CH2 PWM2模式		 
	TIM2->CCMR2|=1<<3; 	    //CH2预装载使能
	TIM2->CCMR2|=6<<12;  	//CH2 PWM2模式		 
	TIM2->CCMR2|=1<<11; 	//CH2预装载使能
		   
	TIM2->CCER|=1<<0;   	//OC2 输出使能
	TIM2->CCER|=1<<4;   	//OC2 输出使能
	TIM2->CCER|=1<<8;   	//OC2 输出使能
	TIM2->CCER|=1<<12;   	//OC2 输出使能
		   
	TIM2->CR1=0x8000;   	//ARPE使能 
	TIM2->CR1|=0x01;    	//使能定时器5 

	PWM_CH1=0;
	PWM_CH2=0;
	PWM_CH3=0;
	PWM_CH4=0;	
									  
} 
void TIM5_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM5时钟使能    
 	TIM5->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
	TIM5->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
	TIM5->DIER|=1<<0;   //允许更新中断	  
	TIM5->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM5_IRQChannel,2);//抢占1，子优先级3，组2									 
}
 	 













