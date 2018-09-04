/**
	*	@brief	config common periph_driver:example GPIO 
	*	@param	None
	*	@retval	None
	*	@periph LED1__PC0		LED2__PD3				@LED
	*					KEY1__PA0		KEY2__PF11			@KEY(KEY connect VCC(3.3V) / KEY1 support WAKEUP)
	*					USART1_TX__PA9		USART1_RX_PA10				@USART1
	*					ADC1_IN1__PA1		ADC1_IN2__PA2		ADC1_IN3__PA3		ADC1_IN4__PA4		@ADC1
	*	  (FULL_REMAP)  PWM1_TIM1_CH1__PE9	PWM2_TIM3_CH1__PC6		@PWM(TIM1/TIM2)
	*/
#ifndef __SYS_H__
#define __SYS_H__

#define FTIR   1  //�½��ش���
#define RTIR   2  //�����ش���
#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "Key.h"
#include "oled.h"
#include "usart3.h"
//#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "I2C_Pro.h"
#include "mpu6050.h"
#include "show.h"					
#include "DataScope_DP.h"

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

extern u8 Way_Angle;                           //��ȡ�Ƕȵ��㷨��2��������  3�������˲�
extern short Encoder_Left,Encoder_Right;       //���ұ��������������
extern int Moto1,Moto2;                        //���PWM���� Ӧ��motor
extern int Voltage,Voltage_Zheng,Voltage_Xiao; //��ص�ѹ������صı���
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
extern int Temperature;
extern float Acceleration_Z;
extern float pitch;

//typedef struct param				//ʹ��λ�� �������б�ʶλ
//{
//	u8 Front:				1;				//ǰ
//	u8 After:				2;				//��
//	u8 Left:				3;				//��
//	u8 Right:				4;				//��
//	u8 Speed:				5;				//�ٶ�
//	u8 Stop:				6;				//�����ͣ
//	u8 Control:			7;				//ң�ط�ʽ
//	u8 LED:					8;				//LED��ʾģʽ
//}Bit_Flag;

typedef struct param				//ʹ��λ�� �������б�ʶλ
{
	u8 Front:				1;				//ǰ
	u8 After:				1;				//��
	u8 Left:				1;				//��
	u8 Right:				1;				//��
	u8 Speed:				1;				//�ٶ�
	u8 Stop:				1;				//�����ͣ
	u8 Control:			1;				//ң�ط�ʽ
	u8 LED:					1;				//LED��ʾģʽ
}Bit_Flag;
/////////////////////////////////////////////////////////////////  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void RCC_Config(void);
void NVIC_Config(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);

#endif













