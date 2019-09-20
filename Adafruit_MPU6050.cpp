
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

boolean Adafruit_MPU6050::begin(uint8_t i2c_address, TwoWire *wire,
                                int32_t sensorID) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensorID);
}

boolean Adafruit_MPU6050::_init(int32_t sensorID) {
  Adafruit_BusIO_Register chip_id =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_WHO_AM_I, 1);

  // make sure we're talking to the right chip
  if (chip_id.read() != MPU6050_DEVICE_ID) {
    return false;
  }

  _sensorid_accel = sensorID;
  _sensorid_gyro = sensorID + 1;
  _sensorid_temp = sensorID + 2;

  Adafruit_BusIO_Register power_mgmt_1 =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_PWR_MGMT_1, 1);

  power_mgmt_1.write(0b10000000); // reset
  while (power_mgmt_1.read() != 0b01000000) { // check for the post reset value
    delay(10);
  }

  Adafruit_BusIO_Register sample_rate_div =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_SMPLRT_DIV, 1);
  sample_rate_div.write(0x00);

  Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_CONFIG, 1);
  config.write(0x00);

  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_GYRO_CONFIG, 1);
  gyro_config.write(0x08);

  Adafruit_BusIO_Register accel_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  accel_config.write(0x00);

  power_mgmt_1.write(0x01); // set clock config to PLL with Gyro X reference

  delay(100);
  return true;
}

/**************************************************************************/
/*!
    @brief Gets the acceleration measurement range.
    @returns  The acceleration measurement range
*/
/**************************************************************************/
mpu6050_accel_range_t Adafruit_MPU6050::getAccelerometerRange(void) {
  Adafruit_BusIO_Register accel_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);
  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config, 2, 3);

  return (mpu6050_accel_range_t)accel_range.read();
}

/**************************************************************************/
/*!
    @brief Sets the accelerometer measurement range
    @param  new_range
            The low range to set. Must be a `mpu6050_accel_range_t`
*/
/**************************************************************************/

void Adafruit_MPU6050::setAccelerometerRange(mpu6050_accel_range_t new_range) {
  Adafruit_BusIO_Register accel_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_ACCEL_CONFIG, 1);

  Adafruit_BusIO_RegisterBits accel_range =
      Adafruit_BusIO_RegisterBits(&accel_config, 2, 3);
  accel_range.write(new_range);
}

mpu6050_gyro_range_t Adafruit_MPU6050::getGyroRange(void){
  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_GYRO_CONFIG, 1);
  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 3);

  return (mpu6050_gyro_range_t)gyro_range.read();
}

void Adafruit_MPU6050::setGyroRange(mpu6050_gyro_range_t new_range){
  Adafruit_BusIO_Register gyro_config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_GYRO_CONFIG, 1);
  Adafruit_BusIO_RegisterBits gyro_range =
      Adafruit_BusIO_RegisterBits(&gyro_config, 2, 3);

  gyro_range.write(new_range);
}

/**************************************************************************/
/*!
    @brief Sets clock source.
    @param  new_clock
            The clock source to set. Must be a `mpu6050_clock_select_t`
*/

/**************************************************************************/
void Adafruit_MPU6050::setClock(mpu6050_clock_select_t new_clock) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits clock_select =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 3, 0);
  clock_select.write(new_clock);
}

mpu6050_clock_select_t Adafruit_MPU6050::getClock(void) {
  Adafruit_BusIO_Register pwr_mgmt =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_PWR_MGMT_1, 1);

  Adafruit_BusIO_RegisterBits clock_select =
      Adafruit_BusIO_RegisterBits(&pwr_mgmt, 3, 0);
  return clock_select.read();
}

void Adafruit_MPU6050::enableGyroX(bool enable){
  Adafruit_BusIO_Register pwr_mgmt_2 =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits gyro_x_en =
    Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 1, 2);

  gyro_x_en.write(enable);
}

bool Adafruit_MPU6050::gyroXEnabled(void){
  Adafruit_BusIO_Register pwr_mgmt_2 =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_PWR_MGMT_2, 1);

  Adafruit_BusIO_RegisterBits gyro_x_en =
    Adafruit_BusIO_RegisterBits(&pwr_mgmt_2, 1, 2);

  return gyro_x_en.read();
}

void Adafruit_MPU6050::read(void) {
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


  mpu6050_accel_range_t accel_range = getAccelerometerRange();

  float accel_scale = 1;
  if (accel_range == MPU6050_RANGE_16_G)
    accel_scale = 2048;
  if (accel_range == MPU6050_RANGE_8_G)
    accel_scale = 4096;
  if (accel_range == MPU6050_RANGE_4_G)
    accel_scale = 8192;
  if (accel_range == MPU6050_RANGE_2_G)
    accel_scale = 16384;

  // setup range dependant scaling
  accX = ((float)rawAccX) / accel_scale;
  accY = ((float)rawAccY) / accel_scale;
  accZ = ((float)rawAccZ) / accel_scale;

  temperature = (rawTemp + 12412.0) / 340.0;
  mpu6050_gyro_range_t gyro_range = getGyroRange();

  float gyro_scale = 1;
  if (gyro_range == MPU6050_RANGE_250_DEG)
    gyro_scale = 131;
  if (gyro_range == MPU6050_RANGE_500_DEG)
    gyro_scale = 65.5;
  if (gyro_range == MPU6050_RANGE_1000_DEG)
    gyro_scale = 32.8;
  if (gyro_range == MPU6050_RANGE_2000_DEG)
    gyro_scale = 16.4;

  Serial.print("gyro scale: "); Serial.println(gyro_scale);
  // TODO: CHeck scaling
  gyroX = ((float)rawGyroX) / gyro_scale;
  gyroY = ((float)rawGyroY) / gyro_scale;
  gyroZ = ((float)rawGyroZ) / gyro_scale;

  // later, set offsets in constructor or something
  // gyroX -= gyroXoffset;
  // gyroY -= gyroYoffset;
  // gyroZ -= gyroZoffset;
}
/*!
*     @brief  Sets the location that the FSYNC pin sample is stored
*     @param  fsync_output
*/
mpu6050_fsync_out_t Adafruit_MPU6050::getFsyncSampleOutput(void){
    Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_CONFIG, 1);
    Adafruit_BusIO_RegisterBits fsync_out =
      Adafruit_BusIO_RegisterBits(&config, 3, 3);
    return fsync_out.read();
}

/*!
*     @brief  Gets the location that the FSYNC pin sample is stored
*     @returns  The current FSYNC output
*/
void Adafruit_MPU6050::setFsyncSampleOutput(mpu6050_fsync_out_t fsync_output){
    Adafruit_BusIO_Register config =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_CONFIG, 1);
    Adafruit_BusIO_RegisterBits fsync_out =
      Adafruit_BusIO_RegisterBits(&config, 3, 3);
    fsync_out.write(fsync_output);
}

/*!
*     @brief  Gets the location that the FSYNC pin sample is stored
*     @returns  The current FSYNC output
*/
void Adafruit_MPU6050::setInterruptPinPolarity(bool active_low){
  Adafruit_BusIO_Register int_pin_config =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_INT_PIN_CONFIG, 1);
  Adafruit_BusIO_RegisterBits int_level =
  Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 7);
  int_level.write(active_low);
}

/*!
*     @brief  Gets the location that the FSYNC pin sample is stored
*     @returns  The current FSYNC output
*/
void Adafruit_MPU6050::setI2CBypass(bool bypass){
  Adafruit_BusIO_Register int_pin_config =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_INT_PIN_CONFIG, 1);
  Adafruit_BusIO_RegisterBits i2c_bypass =
    Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 1);


  Adafruit_BusIO_Register user_ctrl =
    Adafruit_BusIO_Register(i2c_dev, MPU6050_USER_CTRL, 1);
  Adafruit_BusIO_RegisterBits i2c_master_enable =
  Adafruit_BusIO_RegisterBits(&int_pin_config, 1, 5);

  i2c_bypass.write(bypass);
  i2c_master_enable.write(!bypass);
}

void Adafruit_MPU6050::getSensor(sensor_t *accel, sensor_t *gyro, sensor_t *temp) {
  /* Clear the sensor_t object */
  memset(accel, 0, sizeof(sensor_t));

  /* Insert the accel name in the fixed length char array */
  strncpy(accel->name, "MPU6050_A", sizeof(accel->name) - 1);
  accel->name[sizeof(accel->name) - 1] = 0;
  accel->version = 1;
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->min_delay = 0;
  accel->max_value = 0;
  accel->min_value = 0;
  accel->resolution = 0;

  memset(gyro, 0, sizeof(sensor_t));
  strncpy(gyro->name, "MPU6050_G", sizeof(gyro->name) - 1);
  gyro->name[sizeof(gyro->name) - 1] = 0;
  gyro->version = 1;
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->min_delay = 0;
  gyro->max_value = 0.0;  // ToDo
  gyro->min_value = 0.0;  // ToDo
  gyro->resolution = 0.0; // ToDo

  memset(temp, 0, sizeof(sensor_t));
  strncpy(temp->name, "MPU6050_T", sizeof(temp->name) - 1);
  temp->name[sizeof(temp->name) - 1] = 0;
  temp->version = 1;
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->min_delay = 0;
  temp->max_value = 0.0;  // ToDo
  temp->min_value = 0.0;  // ToDo
  temp->resolution = 0.0; // ToDo
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event, Adafruit Unified Sensor format
    @param  event Pointer to an Adafruit Unified sensor_event_t object that
   we'll fill in
    @returns True on successful read
*/
/**************************************************************************/
bool Adafruit_MPU6050::getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
  /* Clear the event */
  memset(accel, 0, sizeof(sensors_event_t));

  accel->version = sizeof(sensors_event_t);
  accel->sensor_id = _sensorid_accel;
  accel->type = SENSOR_TYPE_ACCELEROMETER;
  accel->timestamp = 0;
  // read();

  accel->acceleration.x = accX * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.y = accY * SENSORS_GRAVITY_STANDARD;
  accel->acceleration.z = accZ * SENSORS_GRAVITY_STANDARD;

  /*           GYRO               */
  memset(gyro, 0, sizeof(sensors_event_t));
  gyro->version = sizeof(sensors_event_t);
  gyro->sensor_id = _sensorid_gyro;
  gyro->type = SENSOR_TYPE_GYROSCOPE;
  gyro->timestamp = 0;
  // gyro->gyro.x = gyroData.x * _gyro_dps_digit;
  // gyro->gyro.y = gyroData.y * _gyro_dps_digit;
  // gyro->gyro.z = gyroData.z * _gyro_dps_digit;
  gyro->gyro.x = gyroX;
  gyro->gyro.y = gyroY;
  gyro->gyro.z = gyroZ;

  memset(temp, 0, sizeof(sensors_event_t));
  temp->version   = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = 0;
  temp->temperature = temperature;

  return true;
}
