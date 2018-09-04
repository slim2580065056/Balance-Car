//NVIC_Config(3,3,USART3_IRQn,2);		//BlueTooth中断：：组2(组2中优先级最低的)	
//NVIC_Init(3,2,TIM2_IRQn,2);				//USMART中断：：抢占3，子优先级2，
//NVIC_Config(3,1,TIM2_IRQn,2);			//左编码器->TIM2_IRQn：中断向量分组2，抢占优先级为3，子优先级1
//NVIC_Config(3,0,TIM3_IRQn,2); 		//右编码器->TIM3_IRQn：中断向量分组2，抢占优先级为3，子优先级0
//NVIC_Config(2,2,EXTI0_IRQn,2); 		//SET键->EXTI0_IRQn：中断向量分组2，抢占优先级为2，子优先级2
//NVIC_Config(2,1,EXTI9_5_IRQn,2); 	//BN键->EXTI9_5_IRQn：中断向量分组2，抢占优先级为2，子优先级1	
//NVIC_Config(0,0,TIM1_UP_IRQn,2);			//TIM1更新中断
//NVIC_Config(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)

#include "stm32f10x.h"
#include "usart.h"
#include "sys.h"
#include "Control.h"

#define DATASCOPE		//调试时候上位机显示使用

u8 Way_Angle=2;                             //获取角度的算法，  2：卡尔曼  3：互补滤波 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1;                 //停止标志位和 显示标志位 默认停止 显示打开
short Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u8 delay_50,delay_flag; 
float Acceleration_Z;                      //Z轴加速度计  
float pitch;

int main(void)
{ 
//	RCC_Config();
//	Delay_Config();	    	          //=====延时函数初始化	
//	USART1_Config(115200);	        //=====串口初始化为
//	LED_Init();                     //=====初始化与 LED 连接的硬件接口
//	KEY_Init();                     //=====按键初始化
//	MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口 
//	uart3_init(9600);               //=====串口3初始化
//	printf("usart3 initialized complete\r\n");
//	Encoder_Init_TIM2();            //=====编码器接口
//	Encoder_Init_TIM3();            //=====初始化编码器2
//	//		Adc_Init();                   //=====adc初始化
//	I2C_GPIO_Config();                     //=====IIC初始化
//	MPU6050_initialize();           //=====MPU6050初始化	
//	printf("MPU6050 initialized complete\r\n");
//	DMP_Init();                     //=====初始化DMP 
//	printf("DMP initialized complete\r\n");
	OLED_Init();                    //=====OLED初始化	 
	printf("OLED initialized complete\r\n");		
	TIM1_Config();        					//=====TIM1 5ms定时中断初始化
	printf("system initialized complete\r\n");
	while(1)
	{
		Show_Screen();
		#ifdef DATASCOPE
		DataScope();          //开启MiniBalanceV3.5上位机
		#endif

	} 
}

