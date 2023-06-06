#include "stm32f10x.h"
#include "delay.h"

void delay_init(void)
{
	RCC->APB1ENR |= (1u<<0); // Enable clock for TIM2
	TIM2->ARR = 0xffff-1; //MAX ARR value
	TIM2->PSC = 72-1; // Prescalar value 72Mhz/72 = 1Mhz = 1us
	
	TIM2->CR1 |= (1u<<0);//Enable timer
	while( !(TIM2->SR & (1u<<0))); //wait for the update Flag to set
}

void Delay1Ms(void)
{
	delay_us(1000);// Delay 1ms
}

void delay_us(uint32_t delay)
{
	TIM2->CNT = 0; //RESET counter
	while(TIM2->CNT < delay);//wait to us
}

void Delay_Ms(uint32_t u32DelayInMs)
{
	
	while (u32DelayInMs) {
		Delay1Ms();
		--u32DelayInMs;
	}
}

