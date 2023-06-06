#include <stm32f10x.h>
#include "I2C.h"
#include "delay.h"

GPIO_TypeDef* GPIOx;
uint32_t RCC_APB2Periph_GPIO_x;
uint16_t GPIO_PIN_x, GPIO_PIN_y;

#define SDA_0() GPIO_ResetBits(GPIOx, GPIO_PIN_x)
#define SDA_1() GPIO_SetBits(GPIOx, GPIO_PIN_x)
#define SCL_0() GPIO_ResetBits(GPIOx, GPIO_PIN_y)
#define SCL_1() GPIO_SetBits(GPIOx, GPIO_PIN_y)

void i2c_setup (GPIO_TypeDef* GPIOX, uint16_t GPIO_PIN_X, uint16_t GPIO_PIN_Y , uint32_t RCC_APB2Periph_GPIO_X)
{
	GPIOx = GPIOX;
	GPIO_PIN_x = GPIO_PIN_X;
	GPIO_PIN_y = GPIO_PIN_Y;
	RCC_APB2Periph_GPIO_x = RCC_APB2Periph_GPIO_X;
}

uint8_t SDA_VAL(void)
{
	if (GPIO_ReadInputDataBit(GPIOx, GPIO_PIN_x)){
		return 1;
	}
	return 0;
}

void i2c_init()
{
		
	GPIO_InitTypeDef gpioInit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_x, ENABLE);
	gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
	gpioInit.GPIO_Pin = GPIO_PIN_x | GPIO_PIN_y;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOx, &gpioInit);
	
	SDA_1();
	SCL_1();
}

void i2c_init_read()
{
		
	GPIO_InitTypeDef gpioInit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_x, ENABLE);
	gpioInit.GPIO_Mode = GPIO_Mode_IPD;
	gpioInit.GPIO_Pin = GPIO_PIN_x | GPIO_PIN_y;
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOx, &gpioInit);
	
	SDA_1();
	SCL_1();
}

void i2c_start(void)
{
	
	SCL_1();
	delay_us(3);
	SDA_1();
	delay_us(3);
	SDA_0();
	delay_us(3);
	SCL_0();
	delay_us(3);
}

void i2c_stop(void)
{
	
	SDA_0();
	delay_us(3);
	SCL_1();
	delay_us(3);
	SDA_1();
	delay_us(3);
}

uint8_t i2c_write(uint8_t u8Data)
{
	uint8_t i;
	uint8_t u8Ret;
	
	for (i = 0; i < 8; ++i) {
		if (u8Data & 0x80) {
			SDA_1();
		} else {
			SDA_0();
		}
		delay_us(3);
		SCL_1();
		delay_us(5);
		SCL_0();
		delay_us(2);
		u8Data <<= 1;
	}
	
	SDA_1();
	delay_us(3);
	SCL_1();
	delay_us(3);
	if (SDA_VAL()) {
		u8Ret = 0;
	} else {
		u8Ret = 1;
	}
	delay_us(2);
	SCL_0();
	delay_us(5);
	
	return u8Ret;
}

uint8_t i2c_read(uint8_t u8Ack)
{
	uint8_t i;
	uint8_t u8Ret;
	
	SDA_1();
	delay_us(3);
	
	for (i = 0; i < 8; ++i) {
		u8Ret <<= 1;
		SCL_1();
		delay_us(3);
		if (SDA_VAL()) {
			u8Ret |= 0x01;
		}
		delay_us(2);
		SCL_0();
		delay_us(5);
	}
	
	if (u8Ack) {
		SDA_0();
	} else {
		SDA_1();
	}
	delay_us(3);
	
	SCL_1();
	delay_us(5);
	SCL_0();
	delay_us(5);
	return u8Ret;
}


void My_I2C_Init()
{
	i2c_init();
}

uint8_t I2C_Write(uint8_t Address, uint8_t *pData, uint8_t length)
{
	uint8_t i;
	
	i2c_start();
	if (i2c_write(Address) == 0) {
		i2c_stop();
		return 0;
	}
	for (i = 0; i < length; ++i) {
		if (i2c_write(pData[i]) == 0) {
			i2c_stop();
			return 0;
		}
	}
	i2c_stop();
	
	return 1;
}

uint8_t I2C_Read(uint8_t Address, uint8_t *pData, uint8_t length)
{
	uint8_t i;
	
	i2c_start();
	
	if (i2c_write(Address) == 0) {
		i2c_stop();
		return 0;
	}
	
	for (i = 0; i < length - 1; ++i) {
		pData[i] = i2c_read(1);
	}
	pData[i] = i2c_read(0);
	
	i2c_stop();
	
	return 1;
}

//------------------------------

uint8_t CTR_Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)		     
{
  i2c_start();
	if (i2c_write(SlaveAddress) == 0){
		i2c_stop();
		return 0;
	}
	
	if (i2c_write(REG_Address) == 0){
		i2c_stop();
		return 0;
	}
	
	if (i2c_write(REG_data) == 0){
		i2c_stop();
		return 0;
	}
	
	i2c_stop();
	Delay_Ms(5);
	return 1;
}

uint8_t CTR_Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
	uint8_t putData;
	
	i2c_start();
	if (i2c_write(SlaveAddress) == 0){
		i2c_stop();
		return 0;
	}
	
	if (i2c_write(REG_Address) == 0){
		i2c_stop();
		return 0;
	}
	
	i2c_start();
	
	if (i2c_write(SlaveAddress + 1 ) == 0){
		i2c_stop();
		return 0;
	}
	
	putData = i2c_read(0);
	
	i2c_stop();
	
	return putData;
}			
