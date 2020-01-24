// A demo of the sleep modes for the Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* Create new sensor events */
sensors_event_t a, g, temp;
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
}

/* This example is meant to be used with the serial plotter which makes
 *  it easier to see how the readings change with different settings.
 *  Make sure to poke and prod the sensor while the demo is running to
 *  generate some intersting data!
 */
void loop() {

  /* first show some 'normal' readings */
  mpu.enableSleep(false);
  mpu.enableCycle(false);

  for (uint16_t i = 0; i < 300; i++) {
    printAvailableData();
    delay(10);
  }

  /* Next, turn on cycle mode. Note how this changes how often the
   *  readings are updated.
   *
   *  First set a slow cycle rate so the effect can be seen clearly.
   */

  mpu.setCycleRate(MPU6050_CYCLE_20_HZ);
  /* ensure that sleep mode is not active. Cycle mode only works
   *  as intended while sleep mode is not active */
  mpu.enableSleep(false);
  /* Finally, enable cycle mode */
  mpu.enableCycle(true);

  for (uint16_t i = 0; i < 300; i++) {
    printAvailableData();
    delay(10);
  }

  /* Finally enable sleep mode. Note that while we can still fetch
   *  data from the measurement registers, the measurements are not
   *  updated. In sleep mode the accelerometer and gyroscope are
   *  deactivated to save power, so measurements are halted.
   */
  mpu.enableCycle(false);
  mpu.enableSleep(true);

  for (uint16_t i = 0; i < 300; i++) {
    printAvailableData();
    delay(10);
  }
}

void printAvailableData(void) {

  /* Populate the sensor events with the readings*/
  mpu.getEvent(&a, &g, &temp);

  /* Print out acceleration data in a plotter-friendly format */
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.println("");
}
