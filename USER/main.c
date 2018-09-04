//NVIC_Config(3,3,USART3_IRQn,2);		//BlueTooth�жϣ�����2(��2�����ȼ���͵�)	
//NVIC_Init(3,2,TIM2_IRQn,2);				//USMART�жϣ�����ռ3�������ȼ�2��
//NVIC_Config(3,1,TIM2_IRQn,2);			//�������->TIM2_IRQn���ж���������2����ռ���ȼ�Ϊ3�������ȼ�1
//NVIC_Config(3,0,TIM3_IRQn,2); 		//�ұ�����->TIM3_IRQn���ж���������2����ռ���ȼ�Ϊ3�������ȼ�0
//NVIC_Config(2,2,EXTI0_IRQn,2); 		//SET��->EXTI0_IRQn���ж���������2����ռ���ȼ�Ϊ2�������ȼ�2
//NVIC_Config(2,1,EXTI9_5_IRQn,2); 	//BN��->EXTI9_5_IRQn���ж���������2����ռ���ȼ�Ϊ2�������ȼ�1	
//NVIC_Config(0,0,TIM1_UP_IRQn,2);			//TIM1�����ж�
//NVIC_Config(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)

#include "stm32f10x.h"
#include "usart.h"
#include "sys.h"
#include "Control.h"

#define DATASCOPE		//����ʱ����λ����ʾʹ��

u8 Way_Angle=2;                             //��ȡ�Ƕȵ��㷨��  2��������  3�������˲� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
short Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
u8 delay_50,delay_flag; 
float Acceleration_Z;                      //Z����ٶȼ�  
float pitch;

int main(void)
{ 
//	RCC_Config();
//	Delay_Config();	    	          //=====��ʱ������ʼ��	
//	USART1_Config(115200);	        //=====���ڳ�ʼ��Ϊ
//	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
//	KEY_Init();                     //=====������ʼ��
//	MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� 
//	uart3_init(9600);               //=====����3��ʼ��
//	printf("usart3 initialized complete\r\n");
//	Encoder_Init_TIM2();            //=====�������ӿ�
//	Encoder_Init_TIM3();            //=====��ʼ��������2
//	//		Adc_Init();                   //=====adc��ʼ��
//	I2C_GPIO_Config();                     //=====IIC��ʼ��
//	MPU6050_initialize();           //=====MPU6050��ʼ��	
//	printf("MPU6050 initialized complete\r\n");
//	DMP_Init();                     //=====��ʼ��DMP 
//	printf("DMP initialized complete\r\n");
	OLED_Init();                    //=====OLED��ʼ��	 
	printf("OLED initialized complete\r\n");		
	TIM1_Config();        					//=====TIM1 5ms��ʱ�жϳ�ʼ��
	printf("system initialized complete\r\n");
	while(1)
	{
		Show_Screen();
		#ifdef DATASCOPE
		DataScope();          //����MiniBalanceV3.5��λ��
		#endif

	} 
}

