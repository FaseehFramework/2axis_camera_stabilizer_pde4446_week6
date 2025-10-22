#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h> 

Adafruit_MPU6050 mpu;

float anglePitch = 0; 
unsigned long timer;

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
  
  timer = micros(); 
  Serial.println("---------------------------------");
  Serial.println("Starting filter. Open Serial Plotter to see the angle.");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  unsigned long now = micros();
  float dt = (now - timer) / 1000000.0; 
  timer = now;

  float pitchAcc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;

  float gyroRate = g.gyro.x * 180.0 / M_PI;
  
  anglePitch = 0.98 * (anglePitch + gyroRate * dt) + 0.02 * pitchAcc;

  Serial.println(anglePitch);
  
  delay(10); 
}