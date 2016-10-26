//---------------------------------------------------
//
// SPI.h
// Created 2016-05-13
// Avalon Innovation
//
// Some text explaining the file
//
// TODOS:
//
//---------------------------------------------------


#ifndef IMU_H
#define IMU_H


#include <avr/io.h>
#include <avr/interrupt.h>

void imu_init();
void imu_update();
void imu_calibrate();
void imu_reset();
int16_t imu_getGyroX();
int16_t imu_getGyroY();
int16_t imu_getGyroZ();
int16_t imu_getAccelerometerX();
int16_t imu_getAccelerometerY();
int16_t imu_getAccelerometerZ();
int32_t imu_getAngleX();
int32_t imu_getAngleY();
int32_t imu_getAngleZ();

//Default LSM6DS3 functions
int16_t IMUReadRawAccelX();
int16_t IMUReadRawAccelY();
int16_t IMUReadRawAccelZ();
int16_t IMUReadRawGyroX();
int16_t IMUReadRawGyroY();
int16_t IMUReadRawGyroZ();
float IMUReadFloatAccelX();
float IMUReadFloatAccelY();
float IMUReadFloatAccelZ();
float IMUReadFloatGyroX();
float IMUReadFloatGyroY();
float IMUReadFloatGyroZ();
int16_t IMUReadRawTemp();
float IMUReadTempC();
float IMUReadTempF();
void IMUFifoBegin();
int8_t IMUFifoClear();
int16_t IMUFifoRead();
uint16_t IMUFifoGetStatus();
void IMUFifoEnd();
float IMUCalcGyro(int16_t in);
float IMUCalcAccel(int16_t in);

#endif
