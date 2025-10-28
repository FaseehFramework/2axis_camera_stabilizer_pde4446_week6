#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; // [yaw, pitch, roll]

Servo servo0; // Yaw
Servo servo1; // Pitch (Tilt)
Servo servo2; // Roll

float correct = 0; 
int j = 0; 

#define INTERRUPT_PIN 2
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #endif
    Serial.begin(38400); 
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(-69);
    mpu.setZGyroOffset(27);
    mpu.setZAccelOffset(1551);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    }

    servo0.attach(10);
    servo1.attach(9);
    servo2.attach(8);
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpy.getFIFOCount();

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount(); 
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        if (j <= 300) {
            correct = ypr[0] * 180 / M_PI; 
            j++;
            servo0.write(90);
            servo1.write(90);
            servo2.write(90);
        }
        else {
            float yaw = (ypr[0] * 180 / M_PI) - correct;
            float pitch = ypr[1] * 180 / M_PI;
            float roll = ypr[2] * 180 / M_PI;

            int servo0Value = map(yaw, -90, 90, 180, 0);   // Yaw
            int servo1Value = map(pitch, -90, 90, 180, 0); // Pitch
            int servo2Value = map(roll, -90, 90, 0, 180);  // Roll

            servo0.write(servo0Value);
            servo1.write(servo1Value);
            servo2.write(servo2Value);
        }
    }
}