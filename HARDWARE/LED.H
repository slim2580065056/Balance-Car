/**
	*	@brief  use LED to show Balance Car swerve and brake
	*	@param  None
	*	@retval None
	*/
	
#ifndef __LED_H__
#define __LED_H__

#include "stm32f10x.h"

void LED_Init(void);
void LED_GPIO_Config(void);
void L_LED(u16 set);
void R_LED(u16 set);
void B_LED(u16 set);
void Set_LED(u16 set);

#endif



