#ifndef __ECG_H__
#define __ECG_H__

#include "shared.h"
#include "arm_math.h"

#define StartTim5() TIM5->CR1 |= 0x00000001
#define StartAdc3() ADC3->CR2 |=  ADC_CR2_ADON
#define EnableAdc3Int() ADC3->CR1 |= ADC_CR1_EOCIE

#define StartTim6() TIM6->CR1 |= 0x00000001
#define StopTim6() TIM6->CR1 &= 0xFFFFFFFE
#define ResetTim6() TIM6->CNT = 0
#define GetTim6() TIM6->CNT
#define Tim6_overflow TIM6->SR&0x00000001
#define ResetTim6Flag() TIM6->SR &= 0xFFFFFFFE

extern float ecg_display_values[350];
extern float ecg_display_avg[350];
extern uint16_t current_ecg_position;

extern uint16_t ecg_refresh_request;
extern uint8_t bpm_refresh_request;

extern uint32_t beat_pulse_period;

extern float threshold;

void initEcg(void);

#endif
