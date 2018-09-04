/**
	*	@brief  use LED to show Balance Car swerve and brake
	*	@param  bit set
	*	@retval None
	*/
/*******************************************************************
*	难点：PA15默认为Jlink下载端口中的JTDI，所以先映射为普通IO，再操作
********************************************************************/

#include "LED.h"
#include "sys.h"

/**
	*	@brief  LED Configured Initialized
	*	@param	None
	*	@retval None
	*/
void LED_Init(void)
{
	LED_GPIO_Config();				//转向灯/刹车灯IO控制口初始化
	L_LED(1);
  R_LED(1);
	B_LED(1);
	Set_LED(1);
}
/**
	*	@brief  Config GPIO for LED  -------L_LED->PC13			R_LED->PA12			B_LED->PB5			Set(Key)->PA15   
	*	@param  None
	*	@retval None
	*/
void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC ,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	/*!< JTAG-DP Disabled and SW-DP Enabled ,端口重映射*/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; 	
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; 	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; 	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}
/*************************************************************************************
*	按键编程：i为静态变量，每进入一次自定义函数，++i会自增一次，并与set作比较，
*			当++i==set时，相应的LED翻转，表现为LED灯闪烁；		
*	remark: ++i自增的时间间隔为上一次运行该自定义函数与本次运行自定义函数的时间间隔。	
**************************************************************************************/
/*	左转向灯	*/
void L_LED(u16 set)		//L_LED->PC13
{
	static u16 i;
	if(0==set)	PCout(13)=0;		//关闭L_LED
	else if(1==set)	PCout(13)=1;//开启L_LED
	else	++i==set? i=0,PCout(13)=~PCout(13): i;	//L_LED翻转
}
/*	右转向灯	*/
void R_LED(u16 set)			//R_LED->PA12
{
	static u16 i;
	if(0==set)	PAout(12)=0;
	else if(1==set)	PAout(12)=1;
	else	++i==set? i=0,PAout(12)=~PAout(12): i;	//R_LED翻转
}	
/*	刹车灯	*/
void B_LED(u16 set)			//B_LED->PB5
{
	static u16 i;
	if(0==set)	PBout(5)=0;
	else if(1==set)	PBout(5)=1;
	else ++i==set? i=0,PBout(5)=~PBout(5): i;
	//GPIO_WriteBit(GPIOB, GPIO_Pin_5,(BitAction)set);
}

/*	设置按键指示灯	*/
void Set_LED(u16 set)			//Set(Key)->PA15
{
	static u16 i;
	if(0==set)	PAout(15)=0;
	else if(1==set)	PAout(15)=1;
	else ++i==set? i=0,PAout(15)=~PAout(15): i;
}	


