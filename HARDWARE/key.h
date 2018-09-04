#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f10x.h"
#include "Delay.h"
#include "sys.h"

#define SET_KEY    PAin(8)
#define BN_KEY		 PBin(0)
//�����ֵ
#define KEY_NULL			0
#define KEY_SETVAL		1
#define KEY_BNVAL			2
//���尴��״̬
#define SET_SINGLE		3		//set����
#define SET_LONG			4		//set����

#define BN_SINGLE			5		//bn����
#define BN_LONG				6		//bn������������ģʽ

void KEY_Init(void);
u8 Key_Scan(void);
u8 Get_Key(void);

#endif


//#ifndef __KEY_H__
//#define __KEY_H__	 
//#include "sys.h"
//  /**************************************************************************
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//**************************************************************************/
//#define KEY PAin(8)
//void KEY_Init(void);          //������ʼ��
//u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
//u8 click(void);               //��������ɨ��
//u8 Long_Press(void);           //����ɨ��  
//#endif  
