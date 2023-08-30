#ifndef SR04_H
#define SR04_H
#include "stm32f1xx_hal.h"

//cac trang thai cua chan echo
typedef enum {
	STATE_WAIT_TRIGGER,
	STATE_WAIT_RAISING,
	STATE_WAIT_FALLING,
	STATE_COMPLETE
} STATE_ECHO; 

// 1 doi tuong sr04
typedef struct {
	TIM_HandleTypeDef *htim ;
	GPIO_TypeDef *GPIO_trigger;
	uint16_t GPIO_Pin_trigger;
	GPIO_TypeDef *GPIO_echo;
	uint16_t GPIO_Pin_echo;
}handleTyledef_sr04;


//ham bat chan trigger
void sr04_start_trigger(handleTyledef_sr04 *sr04);
void sr04_handle_interrupt(handleTyledef_sr04 *sr04);
void sr04_handle(handleTyledef_sr04 *sr04);
//ham khoi tao
void sr04_init(handleTyledef_sr04 *sr04,TIM_HandleTypeDef *_htim,GPIO_TypeDef *_GPIO_trigger,uint16_t _GPIO_Pin_trigger,GPIO_TypeDef *_GPIO_echo,uint16_t _GPIO_Pin_echo);
#endif


