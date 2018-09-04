#include "show.h"

unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
u8 Control_Flag;

//--------------�̶���ʾ���֣�����ˢ�£�����CPU����-------------
void Show_Screen(void)
{			
	static u8 n;		//
	//=======================��һ��������ʾBalance_CAR_V3.0=========================
	OLED_S8X16(0,0,"BALANCE_CAR_V3.0");		
	OLED_S6X8(16,0,"=====================");	
	
	//=======================��������ʾ�˲���ң��ģʽ=========================
	if(2==Way_Angle)					OLED_S6X8(24,10,"Kalman");
	else if(3==Way_Angle)   	OLED_S6X8(24,10,"YiJie ");
	if(1==Control_Flag)				OLED_S6X8(24,64,"NRF24L01 ");
	else if(0==Control_Flag)	OLED_S6X8(24,64,"BlueTooth");
	
	//=======================����������ʾ�ٶȣ���������ֵ��=========================
														OLED_S6X8(40,5,"SPEED");
														OLED_C6X8(40,45,'L');
	if( Encoder_Left<0)				OLED_C6X8(40,53,'-'),OLED_S6X8Num(40,61,-Encoder_Left,3,1);
	else 											OLED_C6X8(40,53,'+'),OLED_S6X8Num(40,61,Encoder_Left,3,1);
														OLED_C6X8(40,90,'R');
	if( Encoder_Left<0)				OLED_C6X8(40,98,'-'),OLED_S6X8Num(40,106,-Encoder_Right,3,1);
	else 											OLED_C6X8(40,98,'+'),OLED_S6X8Num(40,106,Encoder_Right,3,1);
	
	//=======================�ڰ�����ʾ�Ƕ�===================================
														OLED_S6X8(56,8,"Angle");
 	if(Angle_Balance<0)				OLED_S6X8Num(56,40,Angle_Balance+360,3,1);
	else					        		OLED_S6X8Num(56,40,Angle_Balance,3,1);
														OLED_S6X8(56,66,"Temp");
														OLED_S6X8Num(56,92,Temperature/10-6,2,0);//-6Ϊƫ��ֵ�����Ĵ�������������¶ȱ�ʵ���¶ȸ�6������
														OLED_S6X8(56,104,".");
														OLED_S6X8Num(56,110,Temperature%10,1,0);
														OLED_C6X8(56,116,123);		//���ASCII��ֵ(�Զ���)
														OLED_C6X8(56,122,'C');
	//APP��λ��ˢ����ʾ											
	if(3==n)		USART_SendData(USART3,(u8)Encoder_Right);
	if(6==n)		USART_SendData(USART3,(u8)Encoder_Left);
	if(9==n)		n=0,USART_SendData(USART3,(u8)((int)Angle_Balance&0xff));//APP��λ��ֻ����8���ֽ�����
	n++;
}

/**************************************************************************
�������ܣ�����ʾ��������λ���������� �ر���ʾ��
��ڲ�������
����  ֵ����
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Angle_Balance, 1 );       //��ʾ�Ƕ� ��λ���ȣ��㣩
		DataScope_Get_Channel_Data( pitch, 2 );         //��ʾ�����������ľ��� ��λ��CM 
		DataScope_Get_Channel_Data( Vol, 3 );                 //��ʾ��ص�ѹ ��λ��V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
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
