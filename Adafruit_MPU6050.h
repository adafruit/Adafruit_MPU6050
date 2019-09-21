/*!
 *  @file Adafruit_MPU6050.h
 *
 * 	I2C Driver for MPU6050 6-Axis Accelerometer and Magnetometer
 *
 * 	This is a library for the Adafruit MPU6050 breakout:
 * 	https://www.adafruit.com/products/3886
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_MPU6050_H
#define _ADAFRUIT_MPU6050_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// TODO: Trust but verify
#define MPU6050_I2CADDR_DEFAULT 0x69 ///< MPU6050 default i2c address w/ AD0 high

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_PIN_CONFIG 0x37
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42
#define MPU6050_ACCEL_OUT 0x3B // base address for sensor data reads
#define MPU6050_DEVICE_ID 0x68 //the correct MPU6050_WHO_AM_I value

/**
 * @brief FSYNC output values
 *
 * Allowed values for `setFsyncSampleOutput`.
 */
typedef enum fsync_out {
  MPU6050_FSYNC_OUT_DISABLED,
  MPU6050_FSYNC_OUT_TEMP,
  MPU6050_FSYNC_OUT_GYROX,
  MPU6050_FSYNC_OUT_GYROY,
  MPU6050_FSYNC_OUT_GYROZ,
  MPU6050_FSYNC_OUT_ACCELX,
  MPU6050_FSYNC_OUT_ACCELY,
  MPU6050_FSYNC_OUT_ACCEL_Z,
} mpu6050_fsync_out_t;  

typedef enum clock_select {
  MPU6050_INTR_8MHz,
  MPU6050_PLL_GYROX,
  MPU6050_PLL_GYROY,
  MPU6050_PLL_GYROZ,
  MPU6050_PLL_EXT_32K,
  MPU6050_PLL_EXT_19MHz,
  MPU6050_STOP = 7,
} mpu6050_clock_select_t;  

/** The accelerometer ranges */
typedef enum  accel_range {
  MPU6050_RANGE_2_G = 0b00,  ///< +/- 2g (default value)
  MPU6050_RANGE_4_G = 0b01,  ///< +/- 4g
  MPU6050_RANGE_8_G = 0b10,  ///< +/- 8g
  MPU6050_RANGE_16_G = 0b11, ///< +/- 16g
} mpu6050_accel_range_t;

/** The accelerometer ranges */
typedef enum gyro_range{
  MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

typedef enum bandwidth {
  MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
  MPU6050_BAND_184_HZ,
  MPU6050_BAND_94_HZ,
  MPU6050_BAND_44_HZ,
  MPU6050_BAND_21_HZ,
  MPU6050_BAND_10_HZ,
  MPU6050_BAND_5_HZ,
} mpu6050_bandwidth_t;

typedef enum cycle_rate {
  MPU6050_CYCLE_1_25_HZ, ///< 1.25 Hz
  MPU6050_CYCLE_5_HZ, ///< 5 Hz
  MPU6050_CYCLE_20_HZ, ///< 20 Hz
  MPU6050_CYCLE_40_HZ, ///< 40 Hz
} mpu6050_cycle_rate_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MPU6050 I2C Digital Potentiometer
 */
class Adafruit_MPU6050 {

public:
  Adafruit_MPU6050();

  boolean begin(uint8_t i2c_addr = MPU6050_I2CADDR_DEFAULT,
                TwoWire *wire = &Wire, int32_t sensorID = 0);

  mpu6050_accel_range_t getAccelerometerRange(void);
  void setAccelerometerRange(mpu6050_accel_range_t);

  mpu6050_gyro_range_t getGyroRange(void);
  void setGyroRange(mpu6050_gyro_range_t);

  // Adafruit_Sensor API/Interface
  void read();
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp);
  void getSensor(sensor_t *accel, sensor_t *gyro, sensor_t *temp);

  void setInterruptPinPolarity(bool active_low);
  void setFsyncSampleOutput(mpu6050_fsync_out_t fsync_output);

  mpu6050_fsync_out_t getFsyncSampleOutput(void);
  void setI2CBypass(bool bypass);

  void setClock(mpu6050_clock_select_t);
  mpu6050_clock_select_t getClock(void);

  void enableGyroX(bool enabled);
  bool gyroXEnabled(void);

  void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
  mpu6050_bandwidth_t getFilterBandwidth(void);

  void setSampleRateDivisor(uint8_t);
  uint8_t getSampleRateDivisor(void);

  void enableSleep(bool enable);
  void enableCycle(bool enable);

  void setCycleRate(mpu6050_cycle_rate_t rate);
  mpu6050_cycle_rate_t getCycleRate(void);

private:
  bool _init(int32_t);

  Adafruit_I2CDevice *i2c_dev;
  float temperature, accX, accY, accZ, gyroX, gyroY, gyroZ;
  int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

  float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY, angleAccZ;

  float accCoef, gyroCoef;

  uint8_t _sensorid_accel, _sensorid_gyro, _sensorid_temp;
};

#endif
