#include "stm32f10x.h"
#include "stdio.h"
#include "math.h" 
#include "i2c_lcd.h"
#include "delay.h"
#include "i2c_mpu6050.h"


float* CTR_MPU6050_getvalue;
char CTR_datasend[32];
uint8_t sw1 = 0;
uint8_t sw2 = 0;
int count = 0;
float Ax = 0, Ay = 0, Az = 0;
float NextAx = 0, NextAy = 0, NextAz = 0;
float variability = 0, threshold = 0.005;
int time = 0;
uint32_t i = 0;

// Khai bao nguyen mau ham
void config_led(void);
uint8_t state_interrupt_sw1(void);
void interrupt_init(uint32_t RCC_APB2Periph_GPIOX ,uint32_t GPIO_Pin_X, uint32_t GPIO_PortSourceGPIOX, 
	                  uint32_t GPIO_PinSourceX, uint32_t EXTI_LineX, uint32_t EXTIX_IRQn,uint8_t NVIC_IRQChannelPreemptionPriorityX);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);


int main(void)
{
	config_led();
	delay_init();
	
	interrupt_init(RCC_APB2Periph_GPIOB, GPIO_Pin_0, GPIO_PortSourceGPIOB, GPIO_PinSource0, 
	EXTI_Line0, EXTI0_IRQn, 0x00);
	
	interrupt_init(RCC_APB2Periph_GPIOB, GPIO_Pin_1, GPIO_PortSourceGPIOB, GPIO_PinSource1, 
	EXTI_Line1, EXTI1_IRQn, 0x01);
	
	I2C_MPU6050_Setup();
  I2C_MPU6050_Init();
	
	I2C_LCD_Setup();
	I2C_LCD_Init();
	I2C_LCD_Clear();
	sprintf(CTR_datasend,"Steps : %d", count);
	I2C_LCD_Puts(CTR_datasend);
	sprintf(CTR_datasend,"Chua hoat dong");
	I2C_LCD_NewLine();
	I2C_LCD_Puts(CTR_datasend);
				
	I2C_MPU6050_Setup();
	CTR_MPU6050_getvalue = CTR_READ_ACCEL_MPU3050();

	Ax = *(CTR_MPU6050_getvalue);
	Ay = *(CTR_MPU6050_getvalue + 1);
	Az = *(CTR_MPU6050_getvalue + 2);
	
	while (1) {
		
		sw1 = state_interrupt_sw1();
		
		if (sw1 == 1)
		{
			GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			I2C_MPU6050_Setup();
			CTR_MPU6050_getvalue = CTR_READ_ACCEL_MPU3050();
			
			NextAx = *(CTR_MPU6050_getvalue);
			NextAy = *(CTR_MPU6050_getvalue + 1);
			NextAz = *(CTR_MPU6050_getvalue + 2);
			
			variability = (fabs(NextAx - Ax) + fabs(NextAy - Ay) + fabs(NextAz - Az) )/3.0;
			
			Ax = NextAx;
			Ay = NextAy;
			Az = NextAz;
			
			if (variability >= threshold) {
				time++;
			} else {
				time = 0;
			}
			
			if (time >= 70) {
				count ++;
				I2C_LCD_Setup();
				I2C_LCD_Clear();
				sprintf(CTR_datasend,"Steps: %d", count);
				I2C_LCD_Puts(CTR_datasend);
				
				time = 0;
			}
			
			if ( i == 300)
			GPIO_SetBits(GPIOA, GPIO_Pin_9);
			if (i == 600){
				GPIO_ResetBits(GPIOA, GPIO_Pin_9);
				i = 0;
			}
			
			i ++;
			Delay_Ms(1);
			
		} else {
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		}
		
	}
}

//Dinh nghia ham
void EXTI0_IRQHandler()
{
		if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
      sw1 ++;
			if (sw1 % 2 == 1)
			{
				sw1 = 1;
			} else {
				sw1 = 0;
			}

				
     EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI1_IRQHandler()
{
		if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
				count = 0;
				I2C_LCD_Setup();
				I2C_LCD_Clear();
				sprintf(CTR_datasend,"Steps: %d", count);
				I2C_LCD_Puts(CTR_datasend);
			
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void interrupt_init(uint32_t RCC_APB2Periph_GPIOX ,uint32_t GPIO_Pin_X, uint32_t GPIO_PortSourceGPIOX, 
	uint32_t GPIO_PinSourceX, uint32_t EXTI_LineX, uint32_t EXTIX_IRQn,uint8_t NVIC_IRQChannelPreemptionPriorityX)
{
    GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Step1: Ennable clock cho GPIOX
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOX, ENABLE);

    // Step2: Config GPIOB_PIN_X as input pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_X;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Step 3: Config ngat ngoài EXTIX
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOX, GPIO_PinSourceX);
		EXTI_InitStructure.EXTI_Line = EXTI_LineX;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
   
    // Step 4: Config uu tiên và cho phép ngat ngoài EXTIX
    NVIC_InitStructure.NVIC_IRQChannel = EXTIX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_IRQChannelPreemptionPriorityX;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


uint8_t state_interrupt_sw1 ()
{
	return sw1;
}

void config_led(void) {
	GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInit);
}
