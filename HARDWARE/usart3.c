#include "usart3.h"
#include "sys.h"
 /**
  ******************************************************************************
  * @file    Bluetooth.c
  * @author  LiuSongHu_QQ2580065056
  * @version V1.0.0
  * @date    2016-12-4
  * @brief   This file provides all the my_usart firmware functions. 
  ******************************************************************************
  * @attention
  *
	*	Use USART3 of APB1	
  ******************************************************************************
  *//**
	*	@brief  USE USART3 Transmite DATA of PC and MCU
	*	@param	None
	*	@retval None
	*	@ramark TX3->PB10    RX3->PB11
	*/
u8 Usart3_Receive;
u8 mode_data[8];
extern Bit_Flag Flag;	//sys.h�ļ��е�λ�����ڶ������б�ʶλ

/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ����� bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 Baudrate)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��UGPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	
	NVIC_Config(3,3,USART3_IRQn,2);				//�ж�������2����ռ���ȼ�3�������ȼ�3���ж���2��������ȼ�
	
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = Baudrate;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}

/***************************************************************************************************************
*		�������������ڴ����жϷ������
*		���룺�������ڵĽ����ж�
*		�����������ٶȿ��ƺͷ������
*		˵����'X'��'Y'���Ʒ�������Ҫ������������'X'��'Y'�������ٶȵ�������������ģʽ
*					����Ϊ������ƣ�����λ���з�����Ƹ�λʱ����һ��'Z'��ֹͣ�ź�
*					Flag_sudu:0->���ٵ���Ĭ��ֵ��			1->���ٵ�
***************************************************************************************************************/
/*  use USART3 of BlueTooth send data	*/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		static	int Rx_data=0;//����������ر���
		Rx_data=USART_ReceiveData(USART3); 
		mode_data[0]=Rx_data;
		if(89==Rx_data && mode_data[2]==mode_data[1] && mode_data[1]==mode_data[0]) //'Y'ֹͣ/����
		{	
			mode_data[2]=0;	mode_data[1]=0;	mode_data[0]=0;	//�����������
			(0==Flag.Speed)?(Flag.Stop=1):(Flag.Speed=0);//3�����ٵ��������ʼ״̬�ڸ��ٵ������Ϊ���ٵ�������رյ��
		}
		if(88==Rx_data && mode_data[2]==mode_data[1] && mode_data[1]==mode_data[0]) //'X'����/�ӵ�
		{	
			mode_data[2]=0;	mode_data[1]=0;	mode_data[0]=0;	//�����������
			(1==Flag.Stop)?(Flag.Stop=0,Flag.Speed=0):(Flag.Speed=1);			//3�����ٵ��������ʼ״̬Ϊֹͣ������������� �����������ٵ�
		}
		//ʹ��appΪ��MiniBalanceV3.5 ����������APP		
		if(Rx_data==90)	Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=0;		//'Z'ֹͣ
		else if(Rx_data==65)	Flag.Front=1,Flag.After=0,Flag.Left=0,Flag.Right=0;//'A'ǰ��
		else if(Rx_data==69)	Flag.Front=0,Flag.After=1,Flag.Left=0,Flag.Right=0;//'E'����
		else if(Rx_data==67)	Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=1;//'C'����
		else if(Rx_data==71)	Flag.Front=0,Flag.After=0,Flag.Left=1,Flag.Right=0;//'G'����
		else Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=0;//ֹͣ

		mode_data[2]=mode_data[1];		//���ݻ�����������ͣ��Ӽ���
		mode_data[1]=mode_data[0];		//��ֹ������������������Ϊ��������������Ч
	}  		
} 


