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
  *	@remark	 		ʹ�����ģ��ķ�ʽ,���͵�ʱ��Ӹ�λ��ʼ����
	*							���������һ������֮�󣬽����������ó����ģʽ
  ******************************************************************************
	*/

#include "I2C_Pro.h"

/**
	*	@brief 	config GPIO
	*	@param 	None
	*	@retval	None
	*/
void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	SCL_H;		//�ͷ�����
	SDA_H;
}

void SDA_IN(void)  		//������Ϊ����ģʽ
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;		//��������ģʽ�����ӻ�������SDA��Ӱ�죬��������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
void SDA_OUT(void) 		//������Ϊ���ģʽ
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;				//��©���ģʽ����ΪI2C�����߽����������裬���Բ���PP
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/**
	*	@brief 	i2c start
	*	@param	None
	*	@retval	None
	*/
void I2C_Start(void)
{
	SDA_OUT();
	SDA_H;
	SCL_H;	
	Delay_us(1);
	SDA_L;
	Delay_us(1);
	SCL_L;		//�������ݱ仯������������ַ
}

/**
	*	@brief 	i2c stop
	*	@param 	None
	*	@retval	None
	*/
void I2C_Stop(void)
{
	SDA_OUT();
	SCL_H;
	SDA_L;
	Delay_us(1);
	SDA_H;
	Delay_us(1);
}

/**
	*	@brief 	i2c_master_ack(FlagStatus)
	*	@param	RESET->noack	/		SET->ack
	*	@retval	None
	*/
void I2C_Master_Ack(FlagStatus k)
{
	SDA_OUT();
	if(k)
	{
		SDA_L;
		SCL_H;
	}
	else
	{
		SDA_H;
		SCL_H;
	}
	Delay_us(1);
	SCL_L;
	SDA_L;
}

/**
	*	@brief 	i2c_check_ack
	*	@param	None
	*	@retval	ErrorStatus
	*/
ErrorStatus I2C_Check_Ack(void)
{
	SDA_IN();
	SCL_H;
	Delay_us(1);		//��������5us
	if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)==RESET)		//PB7�Ƿ�Ϊ�͵�ƽ��Ӧ��
	{
		SCL_L;		//�������ݱ仯�����պ��������
		Delay_us(1);
		return SUCCESS;
	}
	else 		//��Ӧ��
	{
		SDA_L;
		SCL_L;		//�������ݱ仯�����պ��������
		Delay_us(1);
		return	ERROR;
	}
}

/**
	*	@brief 	i2c_send one byte
	*	@param	data
	*	@retval	None
	*	@remark	MSB to LSB
	*/
void I2C_SendByte(u8 data)
{
	u8 i;
	SDA_OUT();
	SDA_H;	//��ֹ���ݱ仯
	for(i=0;i<8;i++)
	{
		SCL_L;		//�������ݱ仯
		((data&0x80)==0x80)?SDA_H:SDA_L;		//������λΪ1��ߵ�ƽ
		Delay_us(1);
		SCL_H;		//��ֹ���ݱ仯		
		Delay_us(1);
		data<<=1;
	}
	SCL_L;			//�������ݱ仯���ж�Ӧ��λ
	Delay_us(1);
}

/**
	*	@brief	read byte
	*	@param	None
	*	@retval	data
	*/
u8 I2C_ReadByte(void)
{
	u8 i,data;
	SDA_IN();	
	for(i=0;i<8;i++)
	{
		SCL_L;		//�������ݱ仯
		Delay_us(1);
		SCL_H;		//��ֹ���ݱ仯
		data<<=1;
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))
			data+=1;
		Delay_us(1);	
	}
	SCL_L;		//�������ݱ仯���ж�Ӧ��
	Delay_us(1);
	return data;
}

/**
	*	@brief 	send the data register
	*	@param	save data register and Pointer to the data
	*	@retval	None
	*/
void I2C_Send_Data(u8 Addr,u8 data)
{
	I2C_Start();								//��ʼ�ź�
	I2C_SendByte(hardware_Addr | 0);			//����������ַ��д->0
	I2C_Check_Ack();						//���Ӧ��
	I2C_SendByte(Addr);					//д���׵�ַ
	I2C_Check_Ack();						//���Ӧ��
	I2C_SendByte(data);						//д������
	I2C_Check_Ack();						//���Ӧ��
	I2C_Stop();									//ֹͣ�ź�
}

/**
	*	@brief 	read the data from register
	*	@param	register address and should read how much byte
	*	@retval Pointer to the data of address
	*/
u8 I2C_Read_Data(u8 Addr)
{
	char p;
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(hardware_Addr | 0);	//������ַ��д->0
	I2C_Check_Ack();			//Ӧ��
	I2C_SendByte(Addr);		//д���׵�ַ
	I2C_Check_Ack();			//���Ӧ��
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(hardware_Addr | 1);	//������ַ����->1
	I2C_Check_Ack();			//���Ӧ��
	p=I2C_ReadByte();			//��������
	I2C_Master_Ack(RESET);	//������Ӧ��
	I2C_Stop();						//ֹͣ�ź�
	return p;
}

/**
	*	@brief 	������дXYZ���������������short I2C_Read_16(u8 REG_Address)��������д����ٶ�
	*	@param	register address
	*	@retval short data
	*/
short I2C_Read_16(u8 Addr)
{
	u16 p;
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(hardware_Addr | 0);	//������ַ��д->0
	I2C_Check_Ack();			//Ӧ��
	I2C_SendByte(Addr);		//д���׵�ַ
	I2C_Check_Ack();			//���Ӧ��
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(hardware_Addr | 1);	//������ַ����->1
	I2C_Check_Ack();			//���Ӧ��
	p=(I2C_ReadByte()&0x00ff)<<8;			//��������
	I2C_Master_Ack(SET);	//����Ӧ��
	p=p | I2C_ReadByte();	//����6bit
	I2C_Master_Ack(RESET);//������Ӧ��
	I2C_Stop();						//ֹͣ�ź�
	return p;
}

/**
	*	@brief  transplant DMP library necessary this function 
	*	@param  addr ,reg ,lenth ,pointer data
	*	@retval yes or no ACK
	*/
u8 I2C_Tx(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	int i,flag;
	I2C_Start();								//��ʼ�ź�
	I2C_SendByte(addr | 0);			//����������ַ��д->0
	flag=I2C_Check_Ack();						//���Ӧ��
	I2C_SendByte(reg);					//д���׵�ַ
	flag=I2C_Check_Ack();						//���Ӧ��
	for(i=0;i<len;i++)
		I2C_SendByte(data[i]);   	//д������		
	flag=I2C_Check_Ack();						//���Ӧ��
	I2C_Stop();									//ֹͣ�ź�
	return flag;
}

u8 I2C_Rx(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	u8 flag;
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(addr | 0);	//������ַ��д->0
	flag=I2C_Check_Ack();			//Ӧ��
	I2C_SendByte(reg);		//д���׵�ַ
	flag=I2C_Check_Ack();			//���Ӧ��
	I2C_Start();					//��ʼ�ź�
	I2C_SendByte(addr | 1);	//������ַ����->1
	flag=I2C_Check_Ack();			//���Ӧ��
	while(len)
	{
		*buf=I2C_ReadByte();			//��������
		I2C_Master_Ack(len==1?RESET:SET);	//������Ӧ��	
		len--;
		buf++;
	}
	I2C_Stop();						//ֹͣ�ź�
	return flag;
}







