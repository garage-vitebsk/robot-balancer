#include <Arduino.h>
#include "IMU6050.h"
#include "L298N.h"

//TODO add configurable pins
DriveSystem *driveSystem = new DriveSystem(new Motor(6, 7, 8),
        new Motor(11, 10, 9));
IMU6050 *imu;

void setup() {
	pinMode(13, OUTPUT);
	digitalWrite(13, 0);
    Serial.begin(19200);
    Serial.println("start");
    imu = new IMU6050();
    Serial.println("connected");
	digitalWrite(13, 1);
}

void loop() {
    imu->process();
    int speed = (int) imu->get_gyro_y_angle() * 10;

	int sign = speed > 0 ? 1 : -1;
	speed *= sign;
    if (speed < 30) {
        speed = 0;
    } else if (speed < 200) {
        speed = 200;
    } else if (speed > 255) {
        speed = 255;
    }
    speed *= sign;
    Serial.println(speed);
    driveSystem->setSpeed(speed, speed);
    // Delay so we don't swamp the serial port
    delay(5);
}
