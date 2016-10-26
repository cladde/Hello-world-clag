//---------------------------------------------------
//
// SIMU.cpp
// Created 2016-05-13
// Avalon Innovation
//
// Some text explaining the file
//
// TODOS:
//
//---------------------------------------------------


#include "IMU.h"
#include "SparkFunLSM6DS3.h"
//#include "USART.h"
//#include "atmega32u4.h"
#include "Time.h"
#include "carSerial.h"

LSM6DS3 IMU;

void imu_init() {
 	carSerial_print("  IMU.begin()...");
	IMU.begin();
	carSerial_print(" OK.\r\n");
	
 	carSerial_print("  IMU.fifoBegin()...");
	IMU.fifoBegin();
	carSerial_print(" OK.\r\n");

 	carSerial_print("  IMU.fifoClear()... ");
	carSerial_print(IMU.fifoClear()?"OK.":"failed!");
	carSerial_print("\r\n");
}

//Values read from LSM6DS3
int16_t gyroXRaw = 0;
int16_t gyroYRaw = 0;
int16_t gyroZRaw = 0;
int16_t accelerometerXRaw = 0;
int16_t accelerometerYRaw = 0;
int16_t accelerometerZRaw = 0;

//Calibration values subtracted from reads
int16_t gyroXOffset = 0;
int16_t gyroYOffset = 0;
int16_t gyroZOffset = 0;
int16_t accelerometerXOffset = 0;
int16_t accelerometerYOffset = 0;
int16_t accelerometerZOffset = 0;

//Corrected values
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
int16_t accelerometerX = 0;
int16_t accelerometerY = 0;
int16_t accelerometerZ = 0;

//Continous integration
//TODO: accelrometer as well (depending on angle)
int32_t angleX = 0;
int32_t angleY = 0;
int32_t angleZ = 0;

void imu_update() {
	uint32_t count = 0;
	if ((IMU.fifoGetStatus() & 0x8000 ) == 0) {
		return;
	} else {
		while ((IMU.fifoGetStatus() & 0x1000 ) == 0) {
			//Gyro X
			gyroXRaw = IMU.fifoRead();
			gyroX = gyroXRaw - gyroXOffset;
			angleX += gyroX;
			//Gyro Y
			gyroYRaw = IMU.fifoRead();
			gyroY = gyroYRaw - gyroYOffset;
			angleY += gyroY;
			//Gyro Z
			gyroZRaw = IMU.fifoRead();
			gyroZ = gyroZRaw - gyroZOffset;
			angleZ += gyroZ;
			//Accelerometer X
			accelerometerXRaw = IMU.fifoRead();
			accelerometerX = accelerometerXRaw - accelerometerXOffset;
			//Accelerometer Y
			accelerometerYRaw = IMU.fifoRead();
			accelerometerY = accelerometerYRaw - accelerometerYOffset;
			//Accelerometer Z
			accelerometerZRaw = IMU.fifoRead();
			accelerometerZ = accelerometerZRaw - accelerometerZOffset;
			
			count += 1;
		}
		//USARTPrintNum(count);
		//USARTWrite('\n');
	}
}

void imu_calibrate() {
	//TODO: Take care of g in Z axis
	uint8_t iterations = 10;
	uint8_t i;
	
	int16_t gyroXSum = 0;
	int16_t gyroYSum = 0;
	int16_t gyroZSum = 0;
	int16_t accelerometerXSum = 0;
	int16_t accelerometerYSum = 0;
	int16_t accelerometerZSum = 0;

	
	for (i = 0; i < iterations; i++) {
		gyroXSum += gyroXRaw;
		gyroYSum += gyroYRaw;
		gyroZSum += gyroZRaw;
		accelerometerXSum += accelerometerXRaw;
		accelerometerYSum += accelerometerYRaw;
		accelerometerZSum += accelerometerZRaw - 2000;
		time_delay(10);
	}
	
	gyroXOffset = gyroXSum / iterations;
	gyroYOffset = gyroYSum / iterations;
	gyroZOffset = gyroZSum / iterations;
	accelerometerXOffset = accelerometerXSum / iterations;
	accelerometerYOffset = accelerometerYSum / iterations;
	accelerometerZOffset = accelerometerZSum / iterations;

}

void imu_reset() {
	gyroX = 0;
	gyroY = 0;
	gyroZ = 0;
	accelerometerX = 0;
	accelerometerY = 0;
	accelerometerZ = 0;
	
	angleX = 0;
	angleY = 0;
	angleZ = 0;
}

int16_t imu_getGyroX() {
	return gyroX;
}

int16_t imu_getGyroY() {
	return gyroY;
}

int16_t imu_getGyroZ() {
	return gyroZ;
}

int16_t imu_getAccelerometerX() {
	return accelerometerX;
}

int16_t imu_getAccelerometerY() {
	return accelerometerY;
}

int16_t imu_getAccelerometerZ() {
	return accelerometerZ;
}

int32_t imu_getAngleX() {
	return angleX;
}

int32_t imu_getAngleY() {
	return angleY;
}

int32_t imu_getAngleZ() {
	return angleZ;
}


//Original functions
int16_t IMUReadRawAccelX() {
	return IMU.readRawAccelX();
}

int16_t IMUReadRawAccelY() {
	return IMU.readRawAccelY();
}

int16_t IMUReadRawAccelZ() {
	return IMU.readRawAccelZ();
}

int16_t IMUReadRawGyroX() {
	return IMU.readRawGyroX();
}

int16_t IMUReadRawGyroY() {
	return IMU.readRawGyroY();
}

int16_t IMUReadRawGyroZ() {
	return IMU.readRawGyroZ();
}

float IMUReadFloatAccelX() {
	return IMU.readFloatAccelX();
}

float IMUReadFloatAccelY() {
	return IMU.readFloatAccelY();
}

float IMUReadFloatAccelZ() {
	return IMU.readFloatAccelZ();
}

float IMUReadFloatGyroX() {
	return IMU.readFloatGyroX();
}

float IMUReadFloatGyroY() {
	return IMU.readFloatGyroY();
}

float IMUReadFloatGyroZ() {
	return IMU.readFloatGyroZ();
}

int16_t IMUReadRawTemp() {
	return IMU.readRawTemp();
}

float IMUReadTempC() {
	return IMU.readTempC();
}

float IMUReadTempF() {
	return IMU.readTempF();
}

void IMUFifoBegin() {
	return IMU.fifoBegin();
}

int8_t IMUFifoClear() {
	return IMU.fifoClear();
}

int16_t IMUFifoRead() {
	return IMU.fifoRead();
}

uint16_t IMUFifoGetStatus() {
	return IMU.fifoGetStatus();
}

void IMUFifoEnd() {
	return IMU.fifoEnd();
}

float IMUCalcGyro(int16_t in) {
	return IMU.calcGyro(in);
}

float IMUCalcAccel(int16_t in) {
	return IMU.calcAccel(in);
}









