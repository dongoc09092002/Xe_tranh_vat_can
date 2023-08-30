#include "sr04.h"
__weak void complete_data_callback(float kc){}

static STATE_ECHO state_echo = STATE_WAIT_TRIGGER;

void sr04_handle(handleTyledef_sr04 *sr04){
	
	if(state_echo == STATE_COMPLETE){
		
		float kc = 0.017*( sr04->htim->Instance->CNT );
		complete_data_callback(kc);
		state_echo = 	STATE_WAIT_TRIGGER;
	}

}

void sr04_start_trigger(handleTyledef_sr04 *sr04){
	
	
	
	if(state_echo == STATE_WAIT_TRIGGER)
	{
		HAL_GPIO_WritePin(sr04->GPIO_trigger , sr04->GPIO_Pin_trigger , 1);
		HAL_Delay(1);
		HAL_GPIO_WritePin(sr04->GPIO_trigger , sr04->GPIO_Pin_trigger , 0);
		state_echo = STATE_WAIT_RAISING;
	}
}

void sr04_handle_interrupt(handleTyledef_sr04 *sr04){
	
	
		switch(state_echo){
			case STATE_WAIT_RAISING:
				if(HAL_GPIO_ReadPin(sr04->GPIO_echo , sr04->GPIO_Pin_echo)==1)
				{	
					sr04->htim->Instance->CNT = 0;
					HAL_TIM_Base_Start(sr04->htim);
					state_echo = STATE_WAIT_FALLING;
				}
				else{
					state_echo = STATE_WAIT_TRIGGER;
				}
					break;
			case STATE_WAIT_FALLING:
				if(HAL_GPIO_ReadPin(sr04->GPIO_echo , sr04->GPIO_Pin_echo) == 0){
					state_echo = STATE_COMPLETE;
					HAL_TIM_Base_Stop(sr04->htim);
				}
				else{
					state_echo = STATE_WAIT_TRIGGER ; 
				}
				break;
			default :
				state_echo = STATE_WAIT_TRIGGER ; 
				break;
		}
}

void sr04_init(handleTyledef_sr04 *sr04,TIM_HandleTypeDef *_htim,GPIO_TypeDef *_GPIO_trigger,uint16_t _GPIO_Pin_trigger,GPIO_TypeDef *_GPIO_echo,uint16_t _GPIO_Pin_echo){
	sr04->htim = _htim;
	sr04->GPIO_trigger = _GPIO_trigger;
	sr04->GPIO_Pin_trigger = _GPIO_Pin_trigger;
	sr04->GPIO_echo = _GPIO_echo;
	sr04->GPIO_Pin_echo = _GPIO_Pin_echo;
}
