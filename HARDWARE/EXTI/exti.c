#include "exti.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "24l01.h"
extern u8 tmp_buf[16];

//外部中断0服务程序
void EXTI0_IRQHandler(void)
{	 
	EXTI->PR=1<<0;  //清除LINE0上的中断标志位  
}
//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	EXTI->PR=1<<2;  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{	

	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)
{	 
//	if(NRF24L01_RxPacket(tmp_buf)==0)//一旦接收到信息,则显示出来.
//	{
//	} 	 
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位  
}		   
void EXTIX_Init(void)
{
	Ex_NVIC_Config(GPIO_G,4,FTIR); 			//下降沿触发

	MY_NVIC_Init(2,2,EXTI4_IRQChannel,2);	//抢占2，子优先级2，组2	   
}












