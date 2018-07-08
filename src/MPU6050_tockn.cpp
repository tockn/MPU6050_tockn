#include "MPU6050_tockn.h"

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
	wire = &w;
	accCoef = aC;
	gyroCoef = gC;
}

void MPU6050::begin(){
	writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
	writeMPU6050(MPU6050_CONFIG, 0x00);
	writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
	
	setAccelSensitivity(0);
	setGyroSensitivity(0);
	this->update();
	
	angleGyroX = 0;
	angleGyroY = 0;
	angleX = this->getAccAngleX();
	angleY = this->getAccAngleY();
	preInterval = millis();
}

void MPU6050::writeMPU6050(uint8_t reg, byte data){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->write(data);
	wire->endTransmission();
}

/**
 *	Write bits to a specific register.
 *
 * @param regAddress Address of the register to write to
 * @param startBit First bit position to write
 * @param length Number of bits to write to
 * @param data Value to write
*/
void MPU6050::writeBits(uint8_t regAddress, uint8_t startBit, uint8_t length, byte data)
{
	uint8_t b = readMPU6050(regAddress);

	uint8_t mask = ((1 << length) - 1) << (startBit - length + 1);
    data <<= (startBit - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
		
	writeMPU6050(regAddress, b);
}

/**
* Reads 1 byte for a specific register of the MPU6050.
*
* @param reg The register to read from
*/
byte MPU6050::readMPU6050(uint8_t reg) 
{
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->endTransmission();
	wire->requestFrom(MPU6050_ADDR, 1);
	
	return wire->read();
}

/**
* Read two bytes from MPU6050. Reads the register specified and the one
* after it.
*
* @param reg Register to read from
*/
int16_t MPU6050::read2BytesMPU6050(byte reg)
{
	return ((int16_t)readMPU6050(reg)) << 8 | readMPU6050(++reg);
}

void MPU6050::setGyroOffsets(float x, float y, float z){
	gyroXoffset = x;
	gyroYoffset = y;
	gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  delay(1000);
	if(console){
    Serial.println();
    Serial.println("========================================");
		Serial.println("calculate gyro offsets");
		Serial.print("DO NOT MOVE A MPU6050");
	}
	for(int i = 0; i < 3000; i++){
		if(console && i % 1000 == 0){
			Serial.print(".");
		}

		rx = getRawGyroX();
		ry = getRawGyroY();
		rz = getRawGyroZ();

		x += rx / gyroSensitivity;
		y += ry / gyroSensitivity;
		z += rz / gyroSensitivity;
	}
	gyroXoffset = x / 3000;
	gyroYoffset = y / 3000;
	gyroZoffset = z / 3000;

	if(console){
    Serial.println();
		Serial.println("Done!!!");
		Serial.print("X : ");Serial.println(gyroXoffset);
		Serial.print("Y : ");Serial.println(gyroYoffset);
		Serial.print("Z : ");Serial.println(gyroZoffset);
		Serial.println("Program will start after 3 seconds");
    Serial.print("========================================");
		delay(3000);
	}
}

void MPU6050::update(void)
{
	float accX = getAccelX();
	float accY = getAccelY();
	float accZ = getAccelY();

	angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
	angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

	float gyroX = getGyroX();
	float gyroY = getGyroY();
	float gyroZ = getGyroZ();

	double interval = (millis() - preInterval) * 0.001;

	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;

	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
	angleZ = angleGyroZ;

	preInterval = millis();
}

/**
* A streamlined version of the MPU6050.update() method. Designed to be used in timer
* interrupts where the delta t between the interrupts is precisely known.
*
* @param interval Delta t between timer interrupts
*/
void MPU6050::interruptUpdate (unsigned long interval)
{
	
}

/** Set full-scale gyro range.
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 *
 * @param range New gyro sensitivity setting
*/
void MPU6050::setGyroSensitivity(uint8_t range)
{
	switch (range)
	{
		case 0:
			gyroSensitivity = MPU6050_250DS_GYRO_SENSITIVITY;
			break;
			
		case 1:
			gyroSensitivity = MPU6050_500DS_GYRO_SENSITIVITY;
			break;
			
		case 2:
			gyroSensitivity = MPU6050_1000DS_GYRO_SENSITIVITY;
			break;
			
		case 3:
			gyroSensitivity = MPU6050_2000DS_GYRO_SENSITIVITY;
			break;
			
		default:
			return;
	}
	
	writeBits(MPU6050_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_SEL_BIT, MPU6050_GYRO_CONFIG_FS_SEL_LENGTH, range);
}

/** Set full-scale accel range.
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 *
 * @param range New accel sensitivity setting
 */
void MPU6050::setAccelSensitivity(uint8_t range)
{
	switch (range)
	{
		case 0:
			accelSensitivity = MPU6050_2G_ACCEL_SENSITIVITY;
			break;
			
		case 1:
			accelSensitivity = MPU6050_4G_ACCEL_SENSITIVITY;
			break;
			
		case 2:
			accelSensitivity = MPU6050_8G_ACCEL_SENSITIVITY;
			break;
			
		case 3:
			accelSensitivity = MPU6050_16G_ACCEL_SENSITIVITY;
			break;
		
		default:
			return;
	}
	
	writeBits(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_FS_SEL_BIT, MPU6050_ACCEL_CONFIG_FS_SEL_LENGTH, range);
}

int16_t MPU6050::getRawAccelX(void)
{
	return read2BytesMPU6050(0x3B);
}

int16_t MPU6050::getRawAccelY(void)
{
	return read2BytesMPU6050(0x3D);
}

int16_t MPU6050::getRawAccelZ(void)
{
	return read2BytesMPU6050(0x3F);
}

int16_t MPU6050::getRawTemp(void)
{
	return read2BytesMPU6050(0x41);
}

int16_t MPU6050::getRawGyroX(void)
{
	return read2BytesMPU6050(0x43);
}

int16_t MPU6050::getRawGyroY(void)
{
	return read2BytesMPU6050(0x45);
}

int16_t MPU6050::getRawGyroZ(void)
{
	return read2BytesMPU6050(0x47);
}

int16_t MPU6050::getAccelX(void)
{
	return getRawAccelX() / accelSensitivity;
}

int16_t MPU6050::getAccelY(void)
{
	return getRawAccelY() / accelSensitivity;
}

int16_t MPU6050::getAccelZ(void)
{
	return getRawAccelZ() / accelSensitivity;
}

int16_t MPU6050::getGyroX(void)
{
	return (getRawGyroX() / gyroSensitivity) - gyroXoffset;
}

int16_t MPU6050::getGyroY(void)
{
	return (getRawGyroY() / gyroSensitivity) - gyroYoffset;
}

int16_t MPU6050::getGyroZ(void)
{
	return (getRawGyroZ() / gyroSensitivity) - gyroZoffset;
}

int16_t MPU6050::getTemp(void)
{
	return (getRawTemp() + 12412.0f) / 340.0f;
}