#ifndef DELAY_H
#define DELAY_H

#include "stm32f10x.h"

void delay_init(void);
void Delay1Ms(void);
void Delay_Ms(uint32_t u32DelayInMs);
void delay_us(uint32_t delay);

#endif
