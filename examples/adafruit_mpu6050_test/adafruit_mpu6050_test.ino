// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 °/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 °/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 °/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 °/s");
    break;
  }
}

void loop() {
  mpu.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  
//  Serial.print(a.acceleration.x);
//  Serial.print(",");
//  Serial.print(a.acceleration.y);
//  Serial.print(",");
//  Serial.print(a.acceleration.z);
//  Serial.print(",");
//  
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.print(",");

  Serial.println(temp.temperature);


  // Serial.print("Accel X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(" m/s^2");
  // Serial.print("\tY: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(" m/s^2 ");
  // Serial.print("\tZ: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.print("Gyro X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(" deg/s");
  // Serial.print("\tY: ");
  // Serial.print(g.gyro.y);
  // Serial.print(" deg/s ");
  // Serial.print("\tZ: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" deg/s");

  delay(100);
}

void loop() {
  mpu.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g);

  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" m/s^2");
  Serial.print("\tY: ");
  Serial.print(a.acceleration.y);
  Serial.print(" m/s^2 ");
  Serial.print("\tZ: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(" deg/s");
  Serial.print("\tY: ");
  Serial.print(g.gyro.y);
  Serial.print(" deg/s ");
  Serial.print("\tZ: ");
  Serial.print(g.gyro.z);
  Serial.println(" deg/s");

  delay(100);
}