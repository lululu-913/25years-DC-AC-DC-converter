#ifndef __OLED_H
#define __OLED_H

#include "F28x_Project.h"
#include "inc/hw_types.h"
#include "iic.h"

// 类型别名定义
typedef unsigned char Uchar;
typedef unsigned int  Uint;
typedef unsigned long Ulong;
typedef unsigned char uchar;
typedef unsigned int  uint;
// 注意：不再定义 uint8_t，hw_types.h 已经定义过了

// 函数声明
void OLED_Init(void);
void OLED_WriteData(Uchar Data);
void OLED_WriteCommand(Uchar Data);
void OLED_SetPos(Uchar x, Uchar page);
void OLED_Clean(void);
void OLED_Fill(void);
void OLED_ShowChar(Uchar x, Uchar y, char chr);
void OLED_ShowString(Uchar x, Uchar y, char *s);
void OLED_ShowNum(Uchar x, Uchar y, int n, Uchar len);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t n);
void num2char(unsigned char *str, double number, uint8_t g, uint8_t l);
void OLED_OFF(void);
void OLED_ON(void);
void OLED_ShowCHinese(Uchar x, Uchar y, Uchar no);
void OLED_DrawPoint(Uchar x, Uchar y);
void OLED_ErasePoint(Uchar x, Uchar y);
void OLED_DrawSin(float32 Freq, Uchar Phi);

#endif
