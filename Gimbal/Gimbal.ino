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
String serialInputString = ""; // To buffer serial input

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
    Serial.println(F("Initializing DMP..."));
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
        Serial.println(F("DMP connection successful."));
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    servo0.attach(10);
    servo1.attach(9);
    servo2.attach(8);
    Serial.println(F("Servos attached. Calibration will run for 300 cycles."));

    Serial.println(F("--- PID Tuning Commands ---"));
    Serial.println(F("Send commands via Serial Monitor (e.g., 'PP 2.5')"));
    Serial.println(F("PP [value] - Set Pitch Kp"));
    Serial.println(F("PI [value] - Set Pitch Ki"));
    Serial.println(F("PD [value] - Set Pitch Kd"));
    Serial.println(F("RP [value] - Set Roll Kp"));
    Serial.println(F("RI [value] - Set Roll Ki"));
    Serial.println(F("RD [value] - Set Roll Kd"));
    Serial.println(F("---------------------------"));

    lastLoopTime = micros(); // Initialize PID loop timer
}

void parseCommand(String cmd) {
    cmd.trim(); // Remove whitespace
    int splitPos = cmd.indexOf(' ');
    if (splitPos == -1) return; // Invalid command format
    
    String command = cmd.substring(0, splitPos);
    command.toUpperCase(); // Case-insensitive commands
    float value = cmd.substring(splitPos + 1).toFloat();

    if (command == F("PP")) {
        pitchKp = value;
        pitchIntegral = 0; // Reset integral on gain change
        Serial.print(F(">>> Set Pitch Kp = ")); Serial.println(pitchKp);
    } else if (command == F("PI")) {
        pitchKi = value;
        pitchIntegral = 0;
        Serial.print(F(">>> Set Pitch Ki = ")); Serial.println(pitchKi);
    } else if (command == F("PD")) {
        pitchKd = value;
        Serial.print(F(">>> Set Pitch Kd = ")); Serial.println(pitchKd);
    } else if (command == F("RP")) {
        rollKp = value;
        rollIntegral = 0;
        Serial.print(F(">>> Set Roll Kp = ")); Serial.println(rollKp);
    } else if (command == F("RI")) {
        rollKi = value;
        rollIntegral = 0;
        Serial.print(F(">>> Set Roll Ki = ")); Serial.println(rollKi);
    } else if (command == F("RD")) {
        rollKd = value;
        Serial.print(F(">>> Set Roll Kd = ")); Serial.println(rollKd);
    } else {
        Serial.println(F(">>> Unknown command."));
    }

    Serial.print(F("Current Pitch[P:")); Serial.print(pitchKp);
    Serial.print(F(",I:")); Serial.print(pitchKi);
    Serial.print(F(",D:")); Serial.print(pitchKd);
    Serial.print(F("] Roll[P:")); Serial.print(rollKp);
    Serial.print(F(",I:")); Serial.print(rollKi);
    Serial.print(F(",D:")); Serial.print(rollKd);
    Serial.println(F("]"));
}

void handleSerialInput() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            parseCommand(serialInputString);
            serialInputString = ""; // Clear for next command
        } else {
            serialInputString += inChar;
        }
    }
}
void loop() {
    handleSerialInput(); // Check for tuning commands first
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
            if (j == 301) { // Print message once calibration is done
                 Serial.println(F("Calibration complete. Starting stabilization."));
            } else {
                 servo0.write(90);
                 servo1.write(90);
                 servo2.write(90);
            }
        }
        else {
            unsigned long now = micros();
            float dt = (now - lastLoopTime) / 1000000.0; 
            lastLoopTime = now;
            if (dt == 0) dt = 1e-6;

            ypr[0] = ypr[0] - correct;
            int servo0Value = map(ypr[0], -90, 90, 0, 180); //yaw
            servo0.write(servo0Value);

            pitchInput = ypr[1] * 180 / M_PI; 
            pitchError = pitchSetpoint - pitchInput; 
            pitchIntegral += pitchError * dt; 
            float pitchDerivative = (pitchError - pitchLastError) / dt; 
            pitchOutput = (pitchKp * pitchError) + (pitchKi * pitchIntegral) + (pitchKd * pitchDerivative);
            pitchLastError = pitchError; 

            rollInput = ypr[2] * 180 / M_PI; 
            rollError = rollSetpoint - rollInput; 
            rollIntegral += rollError * dt; 
            float rollDerivative = (rollError - rollLastError) / dt; 
            rollOutput = (rollKp * rollError) + (rollKi * rollIntegral) + (rollKd * rollDerivative);
            rollLastError = rollError; 
            int servo1Value = 92 + pitchOutput;
            int servo2Value = 78 - rollOutput;
            servo1Value = constrain(servo1Value, 0, 180);
            servo2Value = constrain(servo2Value, 0, 180);
            servo1.write(servo1Value);
            servo2.write(servo2Value);
        }
    }
}