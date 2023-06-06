#ifndef _CTR_MPU6050_
#define _CTR_MPU6050_
#include "i2c.h"

#define MPU6050_ADDR 0xD0

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

void I2C_MPU6050_Setup (void);
void I2C_MPU6050_Init (void);
float* CTR_READ_GYRO_MPU3050(void);
float *CTR_READ_ACCEL_MPU3050(void);
void CTR_GYRO_MPU6050_setup (void);

#endif
