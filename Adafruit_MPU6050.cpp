
/*!
 *  @file Adafruit_MPU6050.cpp
 *
 *  @mainpage Adafruit MPU6050 proximity and ambient light sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the MPU6050 proximity and ambient light sensor library
 * 
 * 	This is a library for the Adafruit MPU6050 breakout:
 * 	https://www.adafruit.com/product/4161
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 * 
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_MPU6050.h"

/*!
 *    @brief  Instantiates a new MPU6050 class
 */
Adafruit_MPU6050::Adafruit_MPU6050(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */

boolean Adafruit_MPU6050::begin(uint8_t i2c_address, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init();
}

boolean Adafruit_MPU6050::_init(void) {
  // Adafruit_BusIO_Register chip_id = 
  //   Adafruit_BusIO_Register(i2c_dev, MPU6050_DEVICE_ID, 2);

  // // make sure we're talking to the right chip
  // if (chip_id.read() != 0x0186) {
  //   return false;
  // }

  // TODO: CHECK CHIP ID
  // writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  Adafruit_BusIO_Register  sample_rate_div = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_SMPLRT_DIV, 1);
  sample_rate_div.write(0x00);

  // writeMPU6050(MPU6050_CONFIG, 0x00);
  Adafruit_BusIO_Register  config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_CONFIG, 1);
  config.write(0x00);
  
  // writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  Adafruit_BusIO_Register  gyro_config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_GYRO_CONFIG, 1);
  gyro_config.write(0x08);

  // writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  Adafruit_BusIO_Register  accel_config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  accel_config.write(0x00);
  
  // writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  Adafruit_BusIO_Register  power_mgmt_1 = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  power_mgmt_1.write(0x01);


  
  // this->update();
  // angleGyroX = 0;
  // angleGyroY = 0;
  // angleX = this->getAccAngleX();
  // angleY = this->getAccAngleY();
  // preInterval = millis();

  return true;
}

// void Adafruit_MPU6050::enableProximityInterrupts(MPU6050_ProximityType interrupt_condition) {
//   Adafruit_BusIO_RegisterBits proximity_int_config = 
//     Adafruit_BusIO_RegisterBits(PS_CONFIG_12, 2, 8);

//   proximity_int_config.write(interrupt_condition);
// }

/**************************************************************************/
/*!
    @brief Gets the proximity low threshold.
    @returns  The current low threshold
*/
/**************************************************************************/
// Adafruit_BusIO_RegisterBits(*register, number_of_bits, shift);
mpu6050_range_t Adafruit_MPU6050::getRange(void){
  Adafruit_BusIO_Register  config =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits range = 
    Adafruit_BusIO_RegisterBits(&config, 2, 3);
  
}
/**************************************************************************/
/*!
    @brief Sets the proximity low threshold.
    @param  low_threshold
            The low threshold to set
*/
/**************************************************************************/


void Adafruit_MPU6050::fetchData(void){
    Adafruit_BusIO_Register data_reg = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_OUT, 14);

    //read 14 bytes = 7x uint16_t

    uint8_t buffer[14];
    // can I make this a int16_t and cast it to a uint8_t?
    data_reg.read(buffer, 14);
    // for (uint8_t i=0; i<14; i++){
    //   Serial.println(buffer[i], BIN);
    // }

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];
    rawTemp = buffer[6] << 8 | buffer[7];
    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

  temp = (rawTemp + 12412.0) / 340.0;

  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;
  ;Serial.print(accX);
  Serial.print(",");Serial.print(accY);
  Serial.print(",");Serial.println(accZ);

}
// void MPU6050::update(){
//   // setup read from 
// 	wire->beginTransmission(MPU6050_ADDR);
// 	wire->write(0x3B);
// 	wire->endTransmission(false);
// 	wire->requestFrom((int)MPU6050_ADDR, 14); 

//   rawAccX = wire->read() << 8 | wire->read();
//   rawAccY = wire->read() << 8 | wire->read();
//   rawAccZ = wire->read() << 8 | wire->read();
//   rawTemp = wire->read() << 8 | wire->read();
//   rawGyroX = wire->read() << 8 | wire->read();
//   rawGyroY = wire->read() << 8 | wire->read();
//   rawGyroZ = wire->read() << 8 | wire->read();

//   temp = (rawTemp + 12412.0) / 340.0;

//   accX = ((float)rawAccX) / 16384.0;
//   accY = ((float)rawAccY) / 16384.0;
//   accZ = ((float)rawAccZ) / 16384.0;

//   angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
//   angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

//   gyroX = ((float)rawGyroX) / 65.5;
//   gyroY = ((float)rawGyroY) / 65.5;
//   gyroZ = ((float)rawGyroZ) / 65.5;

//   gyroX -= gyroXoffset;
//   gyroY -= gyroYoffset;
//   gyroZ -= gyroZoffset;

//   interval = (millis() - preInterval) * 0.001;

//   angleGyroX += gyroX * interval;
//   angleGyroY += gyroY * interval;
//   angleGyroZ += gyroZ * interval;

//   angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
//   angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
//   angleZ = angleGyroZ;

//   preInterval = millis();

// }
