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
extern Bit_Flag Flag;	//sys.h文件中的位域，用于定义所有标识位

/**************************************************************************
函数功能：串口3初始化
入口参数： bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 Baudrate)
{  	 
	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能UGPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	
	NVIC_Config(3,3,USART3_IRQn,2);				//中断向量组2，抢占优先级3，子优先级3，中断组2中最低优先级
	
	//USART3_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  //USART3_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = Baudrate;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure);     //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 
}

/***************************************************************************************************************
*		函数：蓝牙串口传输中断服务程序
*		输入：蓝牙串口的接收中断
*		输出：电机的速度控制和方向控制
*		说明：'X'和'Y'控制方向，且需要连续按下三次'X'或'Y'，进行速度的增减，即调速模式
*					其他为方向控制，在上位机中方向控制复位时发送一个'Z'即停止信号
*					Flag_sudu:0->低速挡（默认值）			1->高速档
***************************************************************************************************************/
/*  use USART3 of BlueTooth send data	*/
void USART3_IRQHandler(void)
{	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		static	int Rx_data=0;//蓝牙接收相关变量
		Rx_data=USART_ReceiveData(USART3); 
		mode_data[0]=Rx_data;
		if(89==Rx_data && mode_data[2]==mode_data[1] && mode_data[1]==mode_data[0]) //'Y'停止/减档
		{	
			mode_data[2]=0;	mode_data[1]=0;	mode_data[0]=0;	//清除缓存数据
			(0==Flag.Speed)?(Flag.Stop=1):(Flag.Speed=0);//3击低速挡，如果初始状态在高速挡，则变为低速挡；否则关闭电机
		}
		if(88==Rx_data && mode_data[2]==mode_data[1] && mode_data[1]==mode_data[0]) //'X'启动/加挡
		{	
			mode_data[2]=0;	mode_data[1]=0;	mode_data[0]=0;	//清除缓存数据
			(1==Flag.Stop)?(Flag.Stop=0,Flag.Speed=0):(Flag.Speed=1);			//3击高速挡，如果初始状态为停止，则启动电机， 否则跳至高速挡
		}
		//使用app为：MiniBalanceV3.5 或蓝牙串口APP		
		if(Rx_data==90)	Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=0;		//'Z'停止
		else if(Rx_data==65)	Flag.Front=1,Flag.After=0,Flag.Left=0,Flag.Right=0;//'A'前进
		else if(Rx_data==69)	Flag.Front=0,Flag.After=1,Flag.Left=0,Flag.Right=0;//'E'后退
		else if(Rx_data==67)	Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=1;//'C'向右
		else if(Rx_data==71)	Flag.Front=0,Flag.After=0,Flag.Left=1,Flag.Right=0;//'G'向左
		else Flag.Front=0,Flag.After=0,Flag.Left=0,Flag.Right=0;//停止

		mode_data[2]=mode_data[1];		//数据互换，处理启停或加减档
		mode_data[1]=mode_data[0];		//防止按键误碰，所以设置为三连按按键才有效
	}  		
} 


