#include "bsp_adc.h"
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
uint16_t GET_ADC(ADC_HandleTypeDef *ADCx, uint32_t ch){
    ADC_ChannelConfTypeDef get_adc_config;
	  get_adc_config.Channel=ch;
	  get_adc_config.Rank=1;
    get_adc_config.SamplingTime= ADC_SAMPLETIME_480CYCLES;
       if (HAL_ADC_ConfigChannel(ADCx, &get_adc_config) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_ADC_Start(ADCx);
		HAL_ADC_PollForConversion(ADCx, 50);
    return (uint16_t)HAL_ADC_GetValue(ADCx);
}
uint16_t GET_ADC_Average(ADC_HandleTypeDef *ADCx,uint8_t ch,uint16_t times){
  uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=GET_ADC(ADCx,ch);
		HAL_Delay(2);
	}
	return temp_val/times;
}
