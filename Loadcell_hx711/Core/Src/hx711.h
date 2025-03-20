#ifndef HX711_H
#define HX711_H

#include "stm32f4xx_hal.h"  // MCU에 맞는 HAL 라이브러리 포함

extern float scale_factor;
extern long offset;  

// 함수 선언
void HX711_Init(void);
long HX711_Read(void);
void delayMicroseconds(uint32_t us);
void HX711_Tare(uint8_t times);
float HX711_GetWeight(uint8_t times);
void HX711_SetScale(float scale);
void HX711_SetOffset(long offset);

#endif // HX711_H
