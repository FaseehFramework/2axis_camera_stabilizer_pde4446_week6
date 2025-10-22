#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  }

  Serial.println("finding MPU6050");

  if (!mpu.begin()) {
    Serial.println("failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 connected");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

}

void loop() {
  // For now, we just confirmed it's working in setup.
  // We'll add data reading in the next commit.
  delay(1000);
  Serial.println("Sensor is working");
}