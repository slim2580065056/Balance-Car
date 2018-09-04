/**
	*	@brief  USE USART3 Transmite DATA of PC and MCU
	*	@param	None
	*	@retval None
	*	@ramark TX3->PB10    RX3->PB11
	*/

#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f10x.h"

void uart3_init(u32 Baudrate);
void USART3_IRQHandler(void);


#endif

