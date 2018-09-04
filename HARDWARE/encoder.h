#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "sys.h"	 
 /**
	*	@brief  encoder generate PWM and use GPIOA and Pin 0/1/6/7 caputure for digital
	*	@param	None
	*	@retval None
	*/
#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
short Read_Encoder(u8 TIMX);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);
#endif
