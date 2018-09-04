/**
	*	breif: Key1 -> PA8 -> Set
	*				 Key2 -> PB0 -> B/N
	**/
#include "Key.h"

u8 SET_Flag=0,BN_Flag=0,S=1;
u16 SPEED_P=360,SPEED_I=3,STAND_P=500,STAND_D=12;
u8 *Control[2]={"BLE ","2.4G"};					//����һ������2��Ԫ�ص�ָ�����飬����ʼ��

void KEY_Init(void)
{
	/*	����TypeDef���ͱ���(��ͬ����uint x,y;)һ�����ȶ����ʹ��	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	/*	����ʱ��	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
	/*	���ݽṹ���еı�����ֵ��Ӧ��ֵ	*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;	//����ƽ����
	/*	��ʼ��IO������ǰ����ΪIO�ڱ�������һ��Ϊָ���IO�ڵ�һ���ṹ��ָ��	*/
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;	//����ƽ����
	/*	��ʼ��IO������ǰ����ΪIO�ڱ�������һ��Ϊָ���IO�ڵ�һ���ṹ��ָ��	*/
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/*	��Ϊ����Ϊ���������Գ�ʼ��Ϊ�ߵ�ƽ	*/
	GPIO_SetBits(GPIOA, GPIO_Pin_8);		
}
/**************************************************************************************
*		������Key_Scan()	->	����ɨ�����ÿ�ж�һ�ξ�ɨ��һ�Σ�����ʱ������������ĿҪ��
*		���룺None			��������ؼ�ֵ	->	���ؼ�ֵ����SET_KEY �� BN_KEY
*		Ϊ����ĵ�����˫����������������׼�����жϻ�׼
**************************************************************************************/
u8 Key_Scan(void)
{
	if(0==SET_KEY || 0==BN_KEY)	//�м�����
	{
		if(0==SET_KEY)	return KEY_SETVAL;
		else if(0==BN_KEY) return KEY_BNVAL;
		else return KEY_NULL;
	}
	else if(1==SET_KEY && 1==BN_KEY)	//�޼�����
		return KEY_NULL;
	else return 0;
}
u8 Get_Key(void)
{
	static u8 set_status=0,bn_status=0;		//�����ͷ�
	static u16 set_count,bn_count;		//key�����������˼��θ��Զ��庯��
	u8 key_temp;			//��ֵ
	key_temp=Key_Scan();	//�Ƿ��м�����
	if(KEY_SETVAL==key_temp)	//SET������
	{
		set_status=1;		//set�������±�ʶ
		set_count++;
	}
	else if(KEY_BNVAL==key_temp)	//BN������
	{
		bn_status=1;		//bn�������±�ʶ
		bn_count++;
	}
	/*		�����Ѿ����£�������ʶ�����Ұ����Ѿ��ͷţ���ֵΪKEY_NULL��		*/
	if((1==set_status || 1==bn_status) && (KEY_NULL==key_temp || KEY_NULL==key_temp))
	{
		set_status=0; bn_status=0;			//�����ͷű�ʶ
		if(set_count>3 && set_count<200)	//set������3*5=15ms  ��  400*5=2000ms=2s
		{
			set_count=0;
			return SET_SINGLE;
		}
		else if(set_count>200)
		{
			set_count=0;
			return SET_LONG;
		}
		else if(bn_count>3 && bn_count<200)	//bn������3*5=15ms  ��  400*5=2000ms=2s 
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












