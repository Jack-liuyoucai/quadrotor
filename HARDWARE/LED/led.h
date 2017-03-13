#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 							  

#define LED_RED_OFF  (GPIOB->ODR |=  (1<<14))
#define LED_RED_ON   (GPIOB->ODR &= ~(1<<14))
#define LED_RED_TST  (GPIOB->ODR ^=  (1<<14))

#define LED_GRE_OFF  (GPIOA->ODR |=  (1<<15))
#define LED_GRE_ON   (GPIOA->ODR &= ~(1<<15))
#define LED_GRE_TST  (GPIOA->ODR ^=  (1<<15))

void LED_Init(void);//³õÊ¼»¯	
void Show_str_leds(void);
void ALL_LEDS_ON(void);
void ALL_LEDS_OFF(void);
void LED_twinkle(void);
	 				    
#endif

















