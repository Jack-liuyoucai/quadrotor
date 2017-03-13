#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

//通过改变TIM3->CCR2的值来改变占空比，从而控制LED0的亮度
#define PWM_CH1 TIM2->CCR1
#define PWM_CH2 TIM2->CCR2 
#define PWM_CH3 TIM2->CCR3 
#define PWM_CH4 TIM2->CCR4     

void TIM3_Int_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM5_Int_Init(u16 arr,u16 psc);
#endif























