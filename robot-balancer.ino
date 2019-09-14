#include <Arduino.h>
#include <AutoPID.h>
#include "IMU6050.h"
#include "L298N.h"

#define LED_PIN 13
#define OUTPUT_MIN -255
#define OUTPUT_MAX 255
#define MAX_DIF 7
#define KP 35
#define KI 9.25
#define KD 52

//TODO add configurable pins
DriveSystem *driveSystem = new DriveSystem(new Motor(6, 7, 8),
        new Motor(11, 10, 9));
IMU6050 *imu;

double angle = 0;
double target = 0;
double speed = 0;

AutoPID myPID = AutoPID(&angle, &target, &speed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
	pinMode(13, OUTPUT);
	digitalWrite(LED_PIN, 0);
  Serial.begin(19200);
//  Serial.println("start");
  imu = new IMU6050();
  myPID.setBangBang(MAX_DIF);
  myPID.setTimeStep(5);
//  Serial.println("connected");
	digitalWrite(LED_PIN, 1);
}

void loop() {
  imu->process();
  angle = imu->get_gyro_y_angle();
  myPID.run();
  driveSystem->setSpeed(-speed, -speed);
  Serial.println(speed);
  digitalWrite(LED_PIN, myPID.atSetPoint(2));
  delay(5);
}
