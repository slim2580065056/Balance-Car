/**
  ******************************************************************************
  * @file    I2C_Pro.c
  * @author  LiuSongHu_QQ2580065056
  * @version V2.0.0
  * @date    2016-11-4
  * @brief   This file provides I2C time series functions. 
  ******************************************************************************
  * @attention	I2C1_SCL->PB6			I2C1_SDA->PB7
	*							���ģʽ��GPIO_Mode_AF_OD
  *	@remark	 		ʹ�����ģ��ķ�ʽ
	*
  ******************************************************************************
	*/

#ifndef __I2C_PRO_H__
#define __I2C_PRO_H__

#include "stm32f10x.h"
#include "Delay.h"

#define SDA_H	GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SDA_L	GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SCL_H	GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define SCL_L	GPIO_ResetBits(GPIOB,GPIO_Pin_6)

#define	hardware_Addr 0xD0	//IICд��ʱ�ĵ�ַ�ֽ�����,MPU6050address is Addr|=(1<<0x68)

//#define SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x0B<<7;}
//#define SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=0x08<<7;}

void I2C_GPIO_Config(void);
void SDA_OUT(void);
void SDA_IN(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Master_Ack(FlagStatus k);
ErrorStatus I2C_Check_Ack(void);
void I2C_SendByte(u8 data);
u8 I2C_ReadByte(void);

void I2C_Send_Data(u8 Addr,u8 data);
u8 I2C_Read_Data(u8 Addr);
short I2C_Read_16(u8 Addr);
u8 I2C_Tx(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
u8 I2C_Rx(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

