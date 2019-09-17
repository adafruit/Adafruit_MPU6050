
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

#include <Adafruit_MPU6050.h>

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

boolean Adafruit_MPU6050::begin(uint8_t i2c_address, TwoWire *wire, int32_t sensorID) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensorID);
}

boolean Adafruit_MPU6050::_init(int32_t sensorID) {
  // Adafruit_BusIO_Register chip_id = 
  //   Adafruit_BusIO_Register(i2c_dev, MPU6050_DEVICE_ID, 2);

  // // make sure we're talking to the right chip
  // if (chip_id.read() != 0x0186) {
  //   return false;
  // }

  _sensorID = sensorID;
  // TODO: CHECK CHIP ID
  Adafruit_BusIO_Register  sample_rate_div = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_SMPLRT_DIV, 1);
  sample_rate_div.write(0x00);

  Adafruit_BusIO_Register  config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_CONFIG, 1);
  config.write(0x00);
  
  Adafruit_BusIO_Register  gyro_config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_GYRO_CONFIG, 1);
  gyro_config.write(0x08);

  Adafruit_BusIO_Register  accel_config = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  accel_config.write(0x00);
  
  Adafruit_BusIO_Register  power_mgmt_1 = 
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  power_mgmt_1.write(0x01);

  // _mpu6050_sensorid_accel = sensorID;
  // _lsm9dso_sensorid_gyro = sensorID + 1;
  // _lsm9dso_sensorid_temp = sensorID + 2;
  
  // _accelSensor = Sensor(this, &Adafruit_MPU6050::readAccel, &Adafruit_MPU6050::getAccelEvent, &     Adafruit_MPU6050::getAccelSensor);
  // _gyroSensor  = Sensor(this, &Adafruit_MPU6050::readGyro,  &Adafruit_MPU6050::getGyroEvent,  &Adafruit_MPU6050::getGyroSensor);
  // _tempSensor  = Sensor(this, &Adafruit_MPU6050::readTemp,  &Adafruit_MPU6050::getTempEvent,  &Adafruit_MPU6050::getTempSensor);
  
  // this->update();
  // angleGyroX = 0;
  // angleGyroY = 0;
  // angleX = this->getAccAngleX();
  // angleY = this->getAccAngleY();
  // preInterval = millis();

  return true;
}

/**************************************************************************/
/*!
    @brief Gets the acceleration measurement range.
    @returns  The acceleration measurement range
*/
/**************************************************************************/
mpu6050_range_t Adafruit_MPU6050::getAccelerometerRange(void){
  Adafruit_BusIO_Register  config =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits accel_range = 
    Adafruit_BusIO_RegisterBits(&config, 2, 3);
  
  return (mpu6050_range_t)accel_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the proximity low threshold.
    @param  low_threshold
            The low threshold to set
*/
/**************************************************************************/
void Adafruit_MPU6050::setAccelerometerRange(mpu6050_range_t new_range){
  Adafruit_BusIO_Register config =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  
  Adafruit_BusIO_RegisterBits accel_range =
    Adafruit_BusIO_RegisterBits(&config, 2, 3);
  accel_range.write(new_range);
}


// void Adafruit_MPU6050::readAccel() {
//   // Read the accelerometer
//   byte buffer[6];
//   readBuffer(XGTYPE, 
//        0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, 
//        6, buffer);
  
//   uint8_t xlo = buffer[0];
//   int16_t xhi = buffer[1];
//   uint8_t ylo = buffer[2];
//   int16_t yhi = buffer[3];
//   uint8_t zlo = buffer[4];
//   int16_t zhi = buffer[5];
  
//   // Shift values to create properly formed integer (low byte first)
//   xhi <<= 8; xhi |= xlo;
//   yhi <<= 8; yhi |= ylo;
//   zhi <<= 8; zhi |= zlo;
//   accelData.x = xhi;
//   accelData.y = yhi;
//   accelData.z = zhi;
// }


void Adafruit_MPU6050::read(void){
    Adafruit_BusIO_Register data_reg = 
      Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_OUT, 14);

    uint8_t buffer[14];
    // can I make this a int16_t and cast it to a uint8_t?
    data_reg.read(buffer, 14);

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];
    rawTemp = buffer[6] << 8 | buffer[7];
    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

  temp = (rawTemp + 12412.0) / 340.0;


  mpu6050_range_t range = getAccelerometerRange();
  float scale = 1;
  if (range == MPU6050_RANGE_16_G)
    scale = 2048;
  if (range == MPU6050_RANGE_8_G)
    scale = 4096;
  if (range == MPU6050_RANGE_4_G)
    scale = 8192;
  if (range == MPU6050_RANGE_2_G)
    scale = 16384;


  //setup range dependant scaling
  accX = ((float)rawAccX) /scale;
  accY = ((float)rawAccY) /scale;
  accZ = ((float)rawAccZ) /scale;


}
// void MPU6050::update(){

//   temp = (rawTemp + 12412.0) / 340.0;

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


void Adafruit_MPU6050::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MPU6050", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->max_value = 0;
  sensor->min_value = 0;
  sensor->resolution = 0;
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  event Pointer to an Adafruit Unified sensor_event_t object that
   we'll fill in
    @returns True on successful read
*/
/**************************************************************************/
bool Adafruit_MPU6050::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;

  return true;
}
/***************************************************************************
 UNIFIED SENSOR FUNCTIONS
 ***************************************************************************/


// /**************************************************************************/
// /*!
//     @brief  Gets the sensor_t device data, Adafruit Unified Sensor format
//     @param  sensor Pointer to an Adafruit Unified sensor_t object that we'll
//    fill in
// */
// /**************************************************************************/
// void Adafruit_LSM9DS1::getSensor(sensor_t *accel, sensor_t *gyro, sensor_t *temp)
// {
//   /* Update appropriate sensor metadata. */
//   if (accel) getAccelSensor(accel);
//   // if (gyro)  getGyroSensor(gyro);
//   // if (temp)  getTempSensor(temp);
// }

// void Adafruit_LSM9DS1::getAccelSensor(sensor_t* sensor) {
//   memset(sensor, 0, sizeof(sensor_t));
//   strncpy (sensor->name, "LSM9DS1_A", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name)- 1] = 0;
//   sensor->version     = 1;
//   sensor->sensor_id   = _lsm9dso_sensorid_accel;
//   sensor->type        = SENSOR_TYPE_ACCELEROMETER;
//   sensor->min_delay   = 0;
//   sensor->max_value   = 0.0;  // ToDo
//   sensor->min_value   = 0.0;  // ToDo
//   sensor->resolution  = 0.0;  // ToDo
// }


// /**************************************************************************/
// /*!
//     @brief  Gets the most recent accel sensor event
// */
// /**************************************************************************/
// bool Adafruit_LSM9DS1::getEvent(sensors_event_t *accelEvent,
//                                 sensors_event_t *magEvent,
//                                 sensors_event_t *gyroEvent,
//                                 sensors_event_t *tempEvent )
// {
//   /* Grab new sensor reading and timestamp. */
//   read();
//   uint32_t timestamp = millis();

//   /* Update appropriate sensor events. */
//   if (accelEvent) getAccelEvent(accelEvent, timestamp);
//   if (magEvent)   getMagEvent(magEvent, timestamp);
//   if (gyroEvent)  getGyroEvent(gyroEvent, timestamp);
//   if (tempEvent)  getTempEvent(tempEvent, timestamp);
  
//   return true;
// }
// void Adafruit_LSM9DS1::getAccelEvent(sensors_event_t* event, uint32_t timestamp) {
//   memset(event, 0, sizeof(sensors_event_t));
//   event->version   = sizeof(sensors_event_t);
//   event->sensor_id = _lsm9dso_sensorid_accel;
//   event->type      = SENSOR_TYPE_ACCELEROMETER;
//   event->timestamp = timestamp;
//   event->acceleration.x = accelData.x * _accel_mg_lsb;
//   event->acceleration.x /= 1000;
//   event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
//   event->acceleration.y = accelData.y * _accel_mg_lsb;
//   event->acceleration.y /= 1000;
//   event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
//   event->acceleration.z = accelData.z * _accel_mg_lsb;
//   event->acceleration.z /= 1000;
//   event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
// }

// void Adafruit_LSM9DS1::getGyroEvent(sensors_event_t* event, uint32_t timestamp) {
//   memset(event, 0, sizeof(sensors_event_t));
//   event->version   = sizeof(sensors_event_t);
//   event->sensor_id = _lsm9dso_sensorid_accel;
//   event->type      = SENSOR_TYPE_GYROSCOPE;
//   event->timestamp = timestamp;
//   event->gyro.x = gyroData.x * _gyro_dps_digit;
//   event->gyro.y = gyroData.y * _gyro_dps_digit;
//   event->gyro.z = gyroData.z * _gyro_dps_digit;
// }
