#ifndef __SIM_IIC_H__
#define __SIM_IIC_H__

#include "stm32f10x.h"
//#include "intrins.h"
#include "sys.h"
#include "Delay.h"
#include "usart.h"
#include "stdio.h"

#define SIM_IIC_OUT	SIM_SCL_OUT(),SIM_SDA_OUT()
#define SIM_IIC_IN  SIM_SCL_OUT(),SIM_SDA_IN()
#define SIM_SCL_H		GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define SIM_SCL_L		GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define SIM_SDA_H		GPIO_SetBits(GPIOB, GPIO_Pin_4)
#define SIM_SDA_L   GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define SIM_SDA_R		GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4)

#define Device_addr		0x78

typedef enum{ACK=0, NACK=1}AckStatus;

void SIM_SCL_OUT(void);
void SIM_SDA_OUT(void);
void SIM_SDA_IN(void);
void SIM_IIC_INIT(void);
void SIM_IIC_START(void);
void SIM_IIC_STOP(void);
AckStatus SIM_IIC_CHECK_ACK(void);
void SIM_IIC_MASTER_ACK(AckStatus flag);
void SIM_SEND_BYTE(u8 byte);
u8 SIM_READ_BYTE(void);
void SIM_SEND_DATA(u8 addr,u8 *P);
u8 *SIM_READ_DATA(u8 addr);
void CMD(u8 cmd);
void DATA(u8 data);


#endif


