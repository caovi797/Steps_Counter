#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"


uint8_t SDA_VAL(void);
void i2c_setup (GPIO_TypeDef* GPIOX, uint16_t GPIO_PIN_X, uint16_t GPIO_PIN_Y , uint32_t RCC_APB2Periph_GPIO_X);

void My_I2C_Init(void);
uint8_t I2C_Write(uint8_t Address, uint8_t *pData, uint8_t length);
uint8_t I2C_Read(uint8_t Address, uint8_t *pData, uint8_t length);
uint8_t CTR_Single_Read(uint8_t SlaveAddress,uint8_t REG_Address);
uint8_t CTR_Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t u8Data);
uint8_t i2c_read(uint8_t u8Ack);

#endif
