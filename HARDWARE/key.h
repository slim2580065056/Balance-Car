#ifndef __KEY_H__
#define __KEY_H__

#include "stm32f10x.h"
#include "Delay.h"
#include "sys.h"

#define SET_KEY    PAin(8)
#define BN_KEY		 PBin(0)
//定义键值
#define KEY_NULL			0
#define KEY_SETVAL		1
#define KEY_BNVAL			2
//定义按键状态
#define SET_SINGLE		3		//set单击
#define SET_LONG			4		//set长按

#define BN_SINGLE			5		//bn单击
#define BN_LONG				6		//bn长按，即连按模式

void KEY_Init(void);
u8 Key_Scan(void);
u8 Get_Key(void);

#endif


//#ifndef __KEY_H__
//#define __KEY_H__	 
//#include "sys.h"
//  /**************************************************************************
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//**************************************************************************/
//#define KEY PAin(8)
//void KEY_Init(void);          //按键初始化
//u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
//u8 click(void);               //单击按键扫描
//u8 Long_Press(void);           //长按扫描  
//#endif  
