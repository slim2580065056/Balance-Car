/**
	*	breif: Key1 -> PA8 -> Set
	*				 Key2 -> PB0 -> B/N
	**/
#include "Key.h"

u8 SET_Flag=0,BN_Flag=0,S=1;
u16 SPEED_P=360,SPEED_I=3,STAND_P=500,STAND_D=12;
u8 *Control[2]={"BLE ","2.4G"};					//定义一个含有2个元素的指针数组，并初始化

void KEY_Init(void)
{
	/*	定义TypeDef类型变量(如同定义uint x,y;)一样，先定义后使用	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*	开启时钟	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
	/*	根据结构体中的变量赋值相应的值	*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;	//将电平拉低
	/*	初始化IO，其中前参数为IO口变量，后一个为指向该IO口的一个结构体指针	*/
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;	//将电平拉低
	/*	初始化IO，其中前参数为IO口变量，后一个为指向该IO口的一个结构体指针	*/
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*	因为按键为下拉，所以初始化为高电平	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_8);		
}
/**************************************************************************************
*		函数：Key_Scan()	->	按键扫描程序，每中断一次就扫描一次，比延时消抖更符合项目要求
*		输入：None			输出：返回键值	->	返回键值，即SET_KEY 或 BN_KEY
*		为下面的单击、双击、长按或连按做准备或判断基准
**************************************************************************************/
u8 Key_Scan(void)
{
	if(0==SET_KEY || 0==BN_KEY)	//有键按下
	{
		if(0==SET_KEY)	return KEY_SETVAL;
		else if(0==BN_KEY) return KEY_BNVAL;
		else return KEY_NULL;
	}
	else if(1==SET_KEY && 1==BN_KEY)	//无键按下
		return KEY_NULL;
	else return 0;
}
u8 Get_Key(void)
{
	static u8 set_status=0,bn_status=0;		//按键释放
	static u16 set_count,bn_count;		//key计数，进入了几次该自定义函数
	u8 key_temp;			//键值
	key_temp=Key_Scan();	//是否有键按下
	if(KEY_SETVAL==key_temp)	//SET键按下
	{
		set_status=1;		//set按键按下标识
		set_count++;
	}
	else if(KEY_BNVAL==key_temp)	//BN键按下
	{
		bn_status=1;		//bn按键按下标识
		bn_count++;
	}
	/*		按键已经按下（按键标识），且按键已经释放（键值为KEY_NULL）		*/
	if((1==set_status || 1==bn_status) && (KEY_NULL==key_temp || KEY_NULL==key_temp))
	{
		set_status=0; bn_status=0;			//按键释放标识
		if(set_count>3 && set_count<200)	//set键持续3*5=15ms  至  400*5=2000ms=2s
		{
			set_count=0;
			return SET_SINGLE;
		}
		else if(set_count>200)
		{
			set_count=0;
			return SET_LONG;
		}
		else if(bn_count>3 && bn_count<200)	//bn键持续3*5=15ms  至  400*5=2000ms=2s 
		{
			bn_count=0;
			return BN_SINGLE;
		}
		else if(bn_count>200)
		{
			bn_count=0;
			return BN_LONG;
		}		
	}
//	return 0;
}












