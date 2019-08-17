#include "Arduino.h"
#include "IMU6050.h"
#include "L298N.h"

//TODO add configurable pins
DriveSystem *driveSystem = new DriveSystem(new Motor(6, 7, 8),
        new Motor(11, 10, 9));
IMU6050 *imu;

void setup() {
    Serial.begin(19200);
    Serial.println("start");
    imu = new IMU6050();
    Serial.println("connected");
}

void loop() {

    imu->process();
    int speed = (int) imu->get_gyro_y_angle() * 10;
//    int speed = 255;
    Serial.println(speed);
    driveSystem->setSpeed(speed, speed);
    // Delay so we don't swamp the serial port
    delay(5);
}

