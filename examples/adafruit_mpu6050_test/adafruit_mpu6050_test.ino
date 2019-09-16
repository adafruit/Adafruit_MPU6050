// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  
  // Try to initialize!
  if (! mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  //mpu.setDataRate(MPU6050_DATARATE_31_25_HZ);
  //Serial.print("Data rate set to: ");
  //switch (mpu.getDataRate()) {
  //  case MPU6050_DATARATE_1_HZ: Serial.println("1 Hz"); break;
  //  case MPU6050_DATARATE_1_95_HZ: Serial.println("1.95 Hz"); break;

  //}
}

void loop() {
  mpu.fetchData();      // get X Y and Z data at once
 
  delay(100); 
}