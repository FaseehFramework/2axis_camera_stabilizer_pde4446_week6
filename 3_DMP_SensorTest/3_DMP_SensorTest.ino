#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
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

Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container

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
    
    Serial.println(F("I2C device setup"));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial.println(F("MPU6050 connection failed"));
        while(1);
    }
    Serial.println(F("MPU6050 connection successful"));

    Serial.println(F("starting DMP"));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP"));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("DMP ready"));
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float yaw   = ypr[0] * 180 / M_PI;
        float pitch = ypr[1] * 180 / M_PI;
        float roll  = ypr[2] * 180 / M_PI;

        Serial.print("YPR: ");
        Serial.print(yaw);
        Serial.print(", ");
        Serial.print(pitch);
        Serial.print(", ");
        Serial.println(roll);
    }
}