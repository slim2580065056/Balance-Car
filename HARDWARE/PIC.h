/*************************************************************************
*		ͼƬ���󣺺���ȡģ���λ���������У������Ҵ��ϵ���
*		ͼƬ�ߴ磺128 * 64
*		��ע��ԭ12864�Ŀ�����������
*************************************************************************/
#ifndef __PIC_H__
#define __PIC_H__
#include "stm32f10x.h"

const u8 KISS[]=  
{
	0xFF,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0xF1,0x09,0x89,0x09,0xF1,0x01,0x01,
	0xF1,0x09,0x89,0x09,0xF1,0x01,0x01,0x01,0x11,0x89,0x49,0x31,0x01,0x79,0x49,0x49,
	0x89,0x01,0xB1,0x49,0x49,0xB1,0x01,0xF1,0x09,0x09,0xF1,0x01,0xF1,0x09,0x09,0xF1,
	0x01,0xF1,0x49,0x49,0x91,0x01,0x79,0x49,0x49,0x89,0x01,0xF1,0x09,0x09,0xF1,0x01,
	0x79,0x49,0x49,0x89,0x01,0xF1,0x49,0x49,0x91,0x01,0x01,0x01,0xF9,0x49,0x49,0xE9,
	0xE9,0xB9,0xB9,0xE9,0xE9,0x49,0x49,0xF9,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0xFF,
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x82,0xC2,0x63,0xE3,0xE2,0x60,
	0xB1,0xD2,0xDA,0xEB,0xEF,0xEE,0xE4,0xF4,0xFF,0xFE,0xFE,0xFE,0xF8,0xF9,0xFA,0xFA,
	0xF1,0xF0,0xF1,0xF2,0xF2,0xE1,0xC0,0xC1,0x82,0x02,0x01,0x00,0x01,0x02,0x02,0x01,
	0x00,0x01,0x02,0x02,0x01,0x00,0x01,0x02,0x02,0x81,0x80,0xC1,0xC2,0xE2,0xE1,0xF0,
	0xF1,0xD2,0xD2,0xD9,0xD8,0xD9,0xDA,0xDA,0xD9,0xD8,0xD8,0xD8,0xDB,0xB2,0xB2,0xA2,
	0xA2,0x63,0x43,0xC2,0xC2,0x82,0x02,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0xC0,0x40,0xFE,0x87,0xF9,0xFC,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xFC,0xF8,0xE0,
	0x80,0x00,0x00,0x00,0x00,0xE0,0xF8,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFE,0xFE,0xFD,0xFD,0xFB,0xF6,0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,
	0xF8,0xFC,0xB7,0xE3,0xF9,0xFC,0xFE,0xFE,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0xFF,0xFF,0x7F,0xF3,0xC3,0x03,0x07,0x07,0x0F,
	0x1F,0x3E,0x00,0x00,0x00,0x01,0x07,0xFF,0x7F,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,
	0xF8,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0x03,0x7F,
	0x7F,0x1F,0x07,0x03,0xC0,0xC0,0xF8,0xF1,0xC0,0xC0,0x41,0x07,0x1E,0x38,0x60,0xC0,
	0x80,0x00,0xC0,0xE0,0x3E,0x1A,0x0F,0x07,0x1E,0x7E,0xD6,0xDE,0x9E,0x9C,0x95,0xBD,
	0xED,0xC9,0x79,0x7B,0x63,0x67,0x6F,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x0F,0x07,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0x83,0x1B,0x89,0x99,0xD1,0xC1,0x07,0x3F,0xFF,0xFF,0xF8,0x80,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
	0xFF,0xFE,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
	0xFF,0x00,0x00,0xC0,0x20,0xC0,0x20,0xC0,0x00,0x20,0xE0,0x20,0x00,0xC0,0x20,0x20,
	0x40,0x00,0xC0,0x23,0x3F,0x4F,0x07,0x1F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xFE,0xFE,0xFE,0xC6,0x00,0x00,0x01,0x03,0x03,0x07,
	0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0x60,0x38,0x1C,
	0x07,0x07,0x1E,0x38,0x60,0xC0,0xC0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x80,0xE0,0xF0,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0xEF,0x03,
	0x00,0x00,0xE0,0x00,0xC0,0x20,0x20,0xC0,0x00,0xE0,0x00,0x00,0xE0,0x00,0x00,0xFF,
	0xFF,0x80,0x80,0x8F,0x80,0x87,0x80,0x8F,0x80,0x88,0x8F,0x88,0x80,0x84,0x89,0x89,
	0x86,0x80,0x84,0x89,0x89,0x86,0x80,0x80,0x80,0xC0,0xE0,0xB1,0x99,0x8D,0x87,0x83,
	0x87,0x9F,0xFF,0xFF,0xBF,0x9F,0x8F,0x9F,0xBF,0xFF,0xAF,0xD6,0xAA,0xD6,0xAA,0xD6,
	0xAA,0xD6,0xAA,0xD6,0xAA,0xD6,0xAA,0xFE,0x83,0x83,0x81,0x80,0x80,0x80,0x80,0x80,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x81,0x81,0x83,0x83,0x82,0x86,0x84,0x8C,0x8C,
	0xE8,0xF8,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xBF,0xBF,
	0xBF,0x9F,0x9F,0xBF,0xEF,0xDF,0xFF,0xE7,0xC3,0x83,0x81,0x80,0x80,0x80,0x80,0x81,
	0x8E,0x81,0x80,0x80,0x87,0x88,0x88,0x87,0x80,0x87,0x88,0x88,0x87,0x80,0x80,0xFF
};

#endif