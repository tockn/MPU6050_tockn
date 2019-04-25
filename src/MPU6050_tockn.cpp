#include "MPU6050_tockn.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
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

void MPU6050::writeMPU6050(byte reg, byte data){
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
}

byte MPU6050::readMPU6050(byte reg) {
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(MPU6050_ADDR, 1);
  byte data =  wire->read();
  return data;
}

void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  delay(delayBefore);
	if(console){
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating gyro offsets");
    Serial.print("DO NOT MOVE MPU6050");
  }
  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
    wire->beginTransmission(MPU6050_ADDR);
		wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int)MPU6050_ADDR, 14, (int)true);

		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
		wire->read() << 8 | wire->read();
    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

		x += ((float)rx) / gyroSensitivity;
		y += ((float)ry) / gyroSensitivity;
		z += ((float)rz) / gyroSensitivity;
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  if(console){
    Serial.println();
    Serial.println("Done!");
    Serial.print("X : ");Serial.println(gyroXoffset);
    Serial.print("Y : ");Serial.println(gyroYoffset);
    Serial.print("Z : ");Serial.println(gyroZoffset);
    Serial.println("Program will start after 3 seconds");
    Serial.print("========================================");
		delay(delayAfter);
	}
}

void MPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read();
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  temp = (rawTemp + 12412.0) / 340.0;

	accX = ((float)rawAccX) / accelSensitivity;
	accY = ((float)rawAccY) / accelSensitivity;
	accZ = ((float)rawAccZ) / accelSensitivity;

  angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

	gyroX = ((float)rawGyroX) / gyroSensitivity;
	gyroY = ((float)rawGyroY) / gyroSensitivity;
	gyroZ = ((float)rawGyroZ) / gyroSensitivity;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis();

}

/**
 *	Write bits to a specific register.
 *
 * @param regAddress Address of the register to write to
 * @param startBit First bit position to write
 * @param length Number of bits to write to
 * @param data Value to write
*/
void MPU6050::writeBits (uint8_t regAddress, uint8_t startBit, uint8_t length, uint8_t data){
	uint8_t b = readMPU6050(regAddress);

	uint8_t mask = ((1 << length) - 1) << (startBit - length + 1);
    data <<= (startBit - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

	writeMPU6050(regAddress, b);
}

/** Set full-scale gyro range.
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 *
 * @param range New gyro sensitivity setting
*/
void MPU6050::setGyroSensitivity (uint8_t range){
	switch (range){
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
void MPU6050::setAccelSensitivity (uint8_t range){
	switch (range){
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
