#include <Servo.h>

Servo servo0; // Yaw
Servo servo1; // Pitch (Tilt)
Servo servo2; // Roll

const int SERVO0_PIN = 10;
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 8;

void setup() {
  Serial.begin(115200);
  Serial.println("Servo Test");

  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  Serial.println("(90 degrees)");
  servo0.write(90);
  servo1.write(90);
  servo2.write(90);
  delay(1000); 
}

void loop() {
  
  Serial.println("Sweeping to 30 degrees");
  servo0.write(30);
  servo1.write(30);
  servo2.write(30);
  delay(1000); 
  Serial.println("Sweeping to 150 degrees");
  servo0.write(150);
  servo1.write(150);
  servo2.write(150);
  delay(1000); 

  Serial.println("Returning to center (90 degrees)");
  servo0.write(90);
  servo1.write(90);
  servo2.write(90);
  delay(1000); 
}