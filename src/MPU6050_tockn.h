#ifndef MPU6050_TOCKN_H
#define MPU6050_TOCKN_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

#define MPU6050_GYRO_CONFIG_FS_SEL_BIT  4
#define MPU6050_GYRO_CONFIG_FS_SEL_LENGTH 2

#define MPU6050_ACCEL_CONFIG_FS_SEL_BIT 4
#define MPU6050_ACCEL_CONFIG_FS_SEL_LENGTH 2

#define MPU6050_2G_ACCEL_SENSITIVITY 16384
#define MPU6050_4G_ACCEL_SENSITIVITY 8192
#define MPU6050_8G_ACCEL_SENSITIVITY 4096
#define MPU6050_16G_ACCEL_SENSITIVITY 2048

#define MPU6050_250DS_GYRO_SENSITIVITY 131
#define MPU6050_500DS_GYRO_SENSITIVITY 65.5f
#define MPU6050_1000DS_GYRO_SENSITIVITY 32.8f
#define MPU6050_2000DS_GYRO_SENSITIVITY 16.4f

class MPU6050{
	public:
	
	MPU6050(TwoWire &w, float aC = 0.02f, float gC = 0.98f);

	void begin();

	void setGyroOffsets(float x, float y, float z);
	
	void setGyroSensitivity (uint8_t range);
	void setAccSensitivity (uint8_t range);

	int16_t getRawAccX(void);
	int16_t getRawAccY(void);
	int16_t getRawAccZ(void);

	int16_t getRawTemp(void);

	int16_t getRawGyroX(void);
	int16_t getRawGyroY(void);
	int16_t getRawGyroZ(void);

	double getTemp(void);

	double getAccX(void);
	double getAccY(void);
	double getAccZ(void);

	double getGyroX(void);
	double getGyroY(void);
	double getGyroZ(void);

	void calcGyroOffsets(bool console = false);

	float getGyroXoffset(){ return gyroXoffset; };
	float getGyroYoffset(){ return gyroYoffset; };
	float getGyroZoffset(){ return gyroZoffset; };

	void update(void);
	void interruptUpdate(unsigned long interval);

	double getGyroAngleX(){ return angleGyroX; };
	double getGyroAngleY(){ return angleGyroY; };
	double getGyroAngleZ(){ return angleGyroZ; };

	double getAngleX(){ return angleX; };
	double getAngleY(){ return angleY; };
	double getAngleZ(){ return angleZ; };

	private:

	TwoWire *wire;

	float gyroXoffset, gyroYoffset, gyroZoffset;
	
	float accSensitivity;
	float gyroSensitivity;

	double angleGyroX, angleGyroY, angleGyroZ;

	double angleX, angleY, angleZ;

	unsigned long preInterval;

	float accCoef, gyroCoef;
	
	void writeBits(uint8_t regAddress, uint8_t startBit, uint8_t length, byte data);
	void writeMPU6050(uint8_t reg, byte data);
	
	byte readMPU6050(uint8_t reg);
	int16_t read2BytesMPU6050(uint8_t reg);
};

#endif