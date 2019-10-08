#include "L298N.h"

//------Motor------
Motor::Motor(int newPwmPin, int newPin1, int newPin2) {
    pwmPin = newPwmPin;
    pin1 = newPin1;
    pin2 = newPin2;

    pinMode(pwmPin, OUTPUT);
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
}

void Motor::setForward() {
    direction = 1;
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
}

void Motor::setReverse() {
    direction = -1;
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, HIGH);
}

void Motor::stop() {
    direction = 0;
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    setSpeed(0);
}

void Motor::setSpeed(int speed) {
    analogWrite(pwmPin, speed);
}

//-----DriveSystem------
DriveSystem::DriveSystem(Motor *newLeftMotor, Motor *newRightMotor) {
    leftMotor = newLeftMotor;
    rightMotor = newRightMotor;
}

int DriveSystem::getLeftSpeed() {
    return leftSpeed;
}

int DriveSystem::getRightSpeed() {
    return rightSpeed;
}

void DriveSystem::setSpeed(int leftSpeed, int rightSpeed) {
    if (leftSpeed == 0) {
        leftMotor->stop();
    } else if (leftSpeed < 0) {
        leftMotor->setReverse();
        leftMotor->setSpeed(-leftSpeed);
    } else {
        leftMotor->setForward();
        leftMotor->setSpeed(leftSpeed);
    }
    if (rightSpeed == 0) {
        rightMotor->stop();
    } else if (rightSpeed < 0) {
        rightMotor->setReverse();
        rightMotor->setSpeed(-rightSpeed);
    } else {
        rightMotor->setForward();
        rightMotor->setSpeed(rightSpeed);
    }

    this->leftSpeed = leftSpeed;
    this->rightSpeed = rightSpeed;
}
