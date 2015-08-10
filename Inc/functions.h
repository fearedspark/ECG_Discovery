#ifndef __FUNCTIONS_H__
#define __FUNCTIONS_H__

#include "shared.h"

#define StartTim2() TIM2->CR1 |= 0x00000001
#define StopTim2() TIM2->CR1 &= 0xFFFFFFFE
#define SetTim2Counter(c) TIM2->CNT = c
#define ResetTim2Flag() TIM2->SR &= 0xFFFFFFFE

void wait(float sec);
void wait_ms(unsigned int ms);
void wait_us(unsigned int us);

#endif
