#include "Pwm.h"
uint16_t dutyToPwm( uint8_t duty , TIM_HandleTypeDef *htim){
	uint16_t result = (uint16_t)(((duty*1.0)/100)*(htim->Instance->ARR));
	return result ;
}

