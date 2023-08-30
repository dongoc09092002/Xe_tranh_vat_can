#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f1xx.h"
//********************CAC TRANG THAI CUA MOTOR************
typedef enum {
	STATE_STOP,     // DUNG LAI
	STATE_FORWARD,	// DONG CO QUAY XUOI
	STATE_BEHIND,   // DONG CO QUAY NGUOC
}MOTOR_STATE;
//********************DOI TUONG MOTOR ********************
typedef struct {
	uint8_t speed;
	uint32_t channel;
	TIM_HandleTypeDef *htim;
	uint32_t GPIO_Pin_output;
	GPIO_TypeDef  *GPIOx_output;
}handle_motor;

void motor_control(handle_motor *motor ,MOTOR_STATE motor_state, uint8_t speed );
void motor_setCCR(handle_motor *motor , uint16_t ccr );
void motor_init(handle_motor *motor,uint32_t _channel,TIM_HandleTypeDef *_htim , uint32_t GPIO_Pin_output , GPIO_TypeDef  *GPIOx_output);
#endif 


