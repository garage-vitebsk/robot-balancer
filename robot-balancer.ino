#include <MPU6050_6Axis_MotionApps20.h>
#include "L298N.h"
#include "PID_v1.h"
#include <math.h>

#define LED_PIN 13
#define DEBUG false

#define OUTPUT_MIN -255
#define OUTPUT_MAX 255
#define TIME_STEP 2
#define ZERO_DIF 3
#define ADJUSTMENT 30
#define KP 25
#define KD 0.2
#define KI 600

DriveSystem *driveSystem = new DriveSystem(new Motor(6, 7, 8),
    new Motor(11, 10, 9));

double angle = 0;
double target = 0;
double speed = 0;

Quaternion q; // [w, x, y, z]
float ypr[3]; // [yaw, pitch, roll]
uint8_t fifoBuffer[64];
uint8_t devStatus; // device status (0 = success, !0 = error)
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
VectorFloat gravity; // [x, y, z] gravity vector
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO

MPU6050 mpu = MPU6050();
PID pid = PID(&angle, &speed, &target, KP, KI, KD, DIRECT);

void setup() {
  Serial.begin(19200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  setupI2C();
  setupImu();
  setupPid();
  digitalWrite(LED_PIN, HIGH);
}

void setupI2C() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void setupImu() {
  mpu.initialize();
#if DEBUG
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  checkImuInitialization();
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
#if DEBUG
  mpu.PrintActiveOffsets();
  Serial.println(F("Enabling DMP..."));
#endif
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void checkImuInitialization() {
  while (devStatus != 0) {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println("). Restart the Arduino");
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  }
}

void setupPid() {
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(TIME_STEP);
  int speedLimit = OUTPUT_MAX - ADJUSTMENT;
  pid.SetOutputLimits(-speedLimit, speedLimit);
}

void loop() {
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    // Interrupt from another event was received. Wait for right interrupt.
    return;
  }
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read a packet from FIFO
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // In case there is > 1 packet available, read more without waiting for an interrupt
      fifoCount -= packetSize;
    }
    readAngle();
    pid.Compute();

    speed = adjustSpeed(speed);
#if DEBUG
    Serial.print(angle);
    Serial.print("\t");
    Serial.println(speed);
#endif
    driveSystem->setSpeed(speed, speed);
    digitalWrite(LED_PIN, fabs(angle) < ZERO_DIF);
    delay(TIME_STEP);
  }
}

void readAngle() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  angle = ypr[1] * 180 / M_PI;
}

double adjustSpeed(double speed) {
  if (speed < -ZERO_DIF) {
    return speed - ADJUSTMENT;
  } else if (speed > ZERO_DIF) {
    return speed + ADJUSTMENT;
  }
  return 0;
}
