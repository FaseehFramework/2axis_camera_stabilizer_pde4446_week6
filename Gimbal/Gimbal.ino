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

float pitchSetpoint = 0.0;
float rollSetpoint = 0.0;
float pitchKp = 1.2;
float pitchKi = 0.0;
float pitchKd = 0.04;
float rollKp = 1.2;
float rollKi = 0.0;
float rollKd = 0.04;
float pitchInput, pitchOutput, pitchError, pitchLastError = 0, pitchIntegral = 0;
float rollInput, rollOutput, rollError, rollLastError = 0, rollIntegral = 0;
unsigned long lastLoopTime;

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
    
    Serial.begin(115200); 
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(1);
    mpu.setYGyroOffset(-35);
    mpu.setZGyroOffset(-21);
    mpu.setZAccelOffset(1749);

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

    lastLoopTime = micros();
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
    fifoCount = mpu.getFIFOCount();

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
            unsigned long now = micros();
            float dt = (now - lastLoopTime) / 1000000.0; 
            lastLoopTime = now;
            if (dt == 0) dt = 1e-6; 

            ypr[0] = ypr[0] - correct;
            int servo0Value = map(ypr[0], -90, 90, 0, 180); //yaw
            servo0.write(servo0Value);

            pitchInput = ypr[1] * 180 / M_PI; // Get sensor reading
            pitchError = pitchSetpoint - pitchInput; // Calculate error
            pitchIntegral += pitchError * dt; // Accumulate integral
            float pitchDerivative = (pitchError - pitchLastError) / dt; // Calculate derivative
            pitchOutput = (pitchKp * pitchError) + (pitchKi * pitchIntegral) + (pitchKd * pitchDerivative);
            pitchLastError = pitchError; // Save error for next loop

            rollInput = ypr[2] * 180 / M_PI; // Get sensor reading
            rollError = rollSetpoint - rollInput; // Calculate error
            rollIntegral += rollError * dt; // Accumulate integral
            float rollDerivative = (rollError - rollLastError) / dt; // Calculate derivative
            rollOutput = (rollKp * rollError) + (rollKi * rollIntegral) + (rollKd * rollDerivative);
            rollLastError = rollError; // Save error for next loop
            

            float pitch_old = ypr[1] * 180 / M_PI;
            float roll_old = ypr[2] * 180 / M_PI;
            int servo1Value = map(pitch_old, -90, 90, 180, 0); 
            int servo2Value = map(roll_old, -90, 90, 0, 180);  
            servo1.write(servo1Value);
            servo2.write(servo2Value);
        }
    }
}