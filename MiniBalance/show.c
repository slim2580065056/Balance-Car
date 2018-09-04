#include "show.h"

unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
u8 Control_Flag;

//--------------固定显示部分，减少刷新，减少CPU负担-------------
void Show_Screen(void)
{			
	static u8 n;		//
	//=======================第一二三行显示Balance_CAR_V3.0=========================
	OLED_S8X16(0,0,"BALANCE_CAR_V3.0");		
	OLED_S6X8(16,0,"=====================");	
	
	//=======================第四行显示滤波及遥控模式=========================
	if(2==Way_Angle)					OLED_S6X8(24,10,"Kalman");
	else if(3==Way_Angle)   	OLED_S6X8(24,10,"YiJie ");
	if(1==Control_Flag)				OLED_S6X8(24,64,"NRF24L01 ");
	else if(0==Control_Flag)	OLED_S6X8(24,64,"BlueTooth");
	
	//=======================第五六行显示速度（编码器数值）=========================
														OLED_S6X8(40,5,"SPEED");
														OLED_C6X8(40,45,'L');
	if( Encoder_Left<0)				OLED_C6X8(40,53,'-'),OLED_S6X8Num(40,61,-Encoder_Left,3,1);
	else 											OLED_C6X8(40,53,'+'),OLED_S6X8Num(40,61,Encoder_Left,3,1);
														OLED_C6X8(40,90,'R');
	if( Encoder_Left<0)				OLED_C6X8(40,98,'-'),OLED_S6X8Num(40,106,-Encoder_Right,3,1);
	else 											OLED_C6X8(40,98,'+'),OLED_S6X8Num(40,106,Encoder_Right,3,1);
	
	//=======================第八行显示角度===================================
														OLED_S6X8(56,8,"Angle");
 	if(Angle_Balance<0)				OLED_S6X8Num(56,40,Angle_Balance+360,3,1);
	else					        		OLED_S6X8Num(56,40,Angle_Balance,3,1);
														OLED_S6X8(56,66,"Temp");
														OLED_S6X8Num(56,92,Temperature/10-6,2,0);//-6为偏移值，即寄存器计算出来的温度比实际温度高6度左右
														OLED_S6X8(56,104,".");
														OLED_S6X8Num(56,110,Temperature%10,1,0);
														OLED_C6X8(56,116,123);		//°的ASCII码值(自定义)
														OLED_C6X8(56,122,'C');
	//APP上位机刷新显示											
	if(3==n)		USART_SendData(USART3,(u8)Encoder_Right);
	if(6==n)		USART_SendData(USART3,(u8)Encoder_Left);
	if(9==n)		n=0,USART_SendData(USART3,(u8)((int)Angle_Balance&0xff));//APP上位机只接收8个字节数据
	n++;
}

/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Angle_Balance, 1 );       //显示角度 单位：度（°）
		DataScope_Get_Channel_Data( pitch, 2 );         //显示超声波测量的距离 单位：CM 
		DataScope_Get_Channel_Data( Vol, 3 );                 //显示电池电压 单位：V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}
