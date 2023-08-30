#include "Motor.h"
#include "Pwm.h"


void motor_init(handle_motor *motor,uint32_t _channel,TIM_HandleTypeDef *_htim , uint32_t GPIO_Pin_output , GPIO_TypeDef  *GPIOx_output){
	motor->channel = _channel ;
	motor->htim = _htim;
	motor->GPIOx_output = GPIOx_output;
	motor->GPIO_Pin_output = GPIO_Pin_output;
}


void motor_setCCR(handle_motor *motor , uint16_t ccr){
	switch(motor->channel){
				case TIM_CHANNEL_1: 
					motor->htim->Instance->CCR1 = ccr;
					break ;
				case TIM_CHANNEL_2: 
					motor->htim->Instance->CCR2 = ccr;
					break ;
				case TIM_CHANNEL_3: 
					motor->htim->Instance->CCR3 = ccr;
					break ;
				case TIM_CHANNEL_4: 
					motor->htim->Instance->CCR4 = ccr;
					break ;
				default : 
					break ;
			}
}

void motor_control(handle_motor *motor ,MOTOR_STATE motor_state, uint8_t speed ){
	//speed : 0-> 100
	uint16_t ccr = dutyToPwm(speed , motor->htim);
	switch(motor_state){
		case STATE_STOP: 
			HAL_GPIO_WritePin(motor->GPIOx_output,motor->GPIO_Pin_output,0);
			motor_setCCR(motor,0);
			break;
		case STATE_FORWARD: 
			HAL_GPIO_WritePin(motor->GPIOx_output,motor->GPIO_Pin_output,1);
			motor_setCCR(motor,motor->htim->Instance->ARR - ccr);
			break;
		case STATE_BEHIND: 
			HAL_GPIO_WritePin(motor->GPIOx_output,motor->GPIO_Pin_output,0);
			motor_setCCR(motor,ccr);
			break;
		default :
			break;
	}
}


