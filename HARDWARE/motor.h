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
	*	IN1    IN2    PWM    STBY			|    O1    O2    ģʽ״̬
	*	 1      1      1/0    1				|     0     0      �ƶ�
	*	 0      1       1     1				|     0     1      ��ת
	*	 0      1       0     1				|     0     0      �ƶ�
	*	 1      0      	1    	1				|     1     0      ��ת
	*	 1      0      	0   	1				|     0     0      �ƶ�
	*	 0      0      	1   	1				|       OFF        ֹͣ
	*	 1/0   1/0     1/0    0				|       OFF        ����
	************************************************************
	*	@Note PWM��Ƶ��������ת���޹ء�Ƶ��Խ�ߵ����Ʋ�ԽС�����ǵ�Դ�����Խ��
	*				���Ե���Ƶ�ʵ���Ҫԭ���ǵ����Ʋ�������Ҫ�������£�PWMƵ���ʵ��͡�
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
