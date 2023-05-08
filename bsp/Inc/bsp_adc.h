#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "main.h"
extern uint16_t GET_ADC(ADC_HandleTypeDef *ADCx, uint32_t ch);
extern uint16_t GET_ADC_Average(ADC_HandleTypeDef *ADCx,uint8_t ch,uint16_t times);



#endif
