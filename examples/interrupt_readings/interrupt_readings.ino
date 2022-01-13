// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Configure MPU6050_INT_PIN according to your hardware: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
#define MPU6050_INT_PIN 2   // On UNO, pins 2 or 3 are available

Adafruit_MPU6050 mpu;
volatile bool MPU6050_DataReady;

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
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set 5 Hz data rate
  mpu.setSampleRateDivisor(200-1);

  // Configure external interrupt
  pinMode(MPU6050_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU6050_INT_PIN), MPU6050_Interrupt, RISING);

  Serial.println("");
  delay(100);

  // Enable data ready interrupt generation
  mpu.enableInterrupt(MPU6050_DATA_RDY_EN);
}

void MPU6050_Interrupt()
{
  MPU6050_DataReady = true;
}

void loop() {

  while(!MPU6050_DataReady);
  MPU6050_DataReady = false;
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}
