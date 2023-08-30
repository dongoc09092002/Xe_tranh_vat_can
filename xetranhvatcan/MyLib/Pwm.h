#ifndef PWM_H
#define PWM_H
#include "stm32f1xx.h"
uint16_t dutyToPwm( uint8_t duty , TIM_HandleTypeDef *htim);
#endif 
