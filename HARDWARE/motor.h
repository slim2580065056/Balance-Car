#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <sys.h>	 
/**
	*	@brief  USE chip of TB6612FNG to control motor move 
	*	@param  None
	*	@retval None
	* @remark 
	************************************************************
	*           INPUT               |         OUTPUT
	*	IN1    IN2    PWM    STBY			|    O1    O2    模式状态
	*	 1      1      1/0    1				|     0     0      制动
	*	 0      1       1     1				|     0     1      反转
	*	 0      1       0     1				|     0     0      制动
	*	 1      0      	1    	1				|     1     0      正转
	*	 1      0      	0   	1				|     0     0      制动
	*	 0      0      	1   	1				|       OFF        停止
	*	 1/0   1/0     1/0    0				|       OFF        待机
	************************************************************
	*	@Note PWM的频率与电机的转速无关。频率越高电流纹波越小，但是电源的损耗越大；
	*				所以调节频率的主要原则是电流纹波在满足要求的情况下，PWM频率适当低。
	*	@GPIO_Pin map    
	*								 STBY->PB0
	*			Left			 AIN1->PA3    AIN2->PA2       PWM1->PA8->TIM1_CH1
	*			Right      BIN1->PA4    BIN2->PA5       PWM2->PA11->TIM1_CH4
	*/
	
#define AIN1   PAout(5)
#define AIN2   PAout(4)
#define BIN1   PAout(3)
#define BIN2   PAout(2)
#define PWMA   TIM4->CCR3  //PB8	
#define PWMB   TIM4->CCR4  //PB9
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
