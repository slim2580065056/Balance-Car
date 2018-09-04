/**
	*	@brief  use LED to show Balance Car swerve and brake
	*	@param  bit set
	*	@retval None
	*/
/*******************************************************************
*	�ѵ㣺PA15Ĭ��ΪJlink���ض˿��е�JTDI��������ӳ��Ϊ��ͨIO���ٲ���
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
	LED_GPIO_Config();				//ת���/ɲ����IO���ƿڳ�ʼ��
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
	
	/*!< JTAG-DP Disabled and SW-DP Enabled ,�˿���ӳ��*/
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
*	������̣�iΪ��̬������ÿ����һ���Զ��庯����++i������һ�Σ�����set���Ƚϣ�
*			��++i==setʱ����Ӧ��LED��ת������ΪLED����˸��		
*	remark: ++i������ʱ����Ϊ��һ�����и��Զ��庯���뱾�������Զ��庯����ʱ������	
**************************************************************************************/
/*	��ת���	*/
void L_LED(u16 set)		//L_LED->PC13
{
	static u16 i;
	if(0==set)	PCout(13)=0;		//�ر�L_LED
	else if(1==set)	PCout(13)=1;//����L_LED
	else	++i==set? i=0,PCout(13)=~PCout(13): i;	//L_LED��ת
}
/*	��ת���	*/
void R_LED(u16 set)			//R_LED->PA12
{
	static u16 i;
	if(0==set)	PAout(12)=0;
	else if(1==set)	PAout(12)=1;
	else	++i==set? i=0,PAout(12)=~PAout(12): i;	//R_LED��ת
}	
/*	ɲ����	*/
void B_LED(u16 set)			//B_LED->PB5
{
	static u16 i;
	if(0==set)	PBout(5)=0;
	else if(1==set)	PBout(5)=1;
	else ++i==set? i=0,PBout(5)=~PBout(5): i;
	//GPIO_WriteBit(GPIOB, GPIO_Pin_5,(BitAction)set);
}

/*	���ð���ָʾ��	*/
void Set_LED(u16 set)			//Set(Key)->PA15
{
	static u16 i;
	if(0==set)	PAout(15)=0;
	else if(1==set)	PAout(15)=1;
	else ++i==set? i=0,PAout(15)=~PAout(15): i;
}	

