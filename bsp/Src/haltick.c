#include "haltick.h"
static uint32_t micros(void){
  uint32_t haltick = 0;
	register uint32_t ms=0,us=0;
	ms=HAL_GetTick();
  us=TIM2->CNT;
  return haltick=ms*1000+us;
}
void delay_ms(uint32_t ms){
  uint32_t now = HAL_GetTick();
  while(micros()-now < ms);
}

void delay_us(uint32_t us){
  uint32_t now = micros();
  while(micros()-now < us);
}


