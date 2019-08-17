#include "L298N.h"

//------Motor------
Motor::Motor(int newPwmPin, int newPin1, int newPin2) {
    pwmPin = newPwmPin;
    pin1 = newPin1;
    pin2 = newPin2;
    // СѓС‚СЃР°РЅРѕРІРєР° СЂРµР¶РёРјР° РїРѕСЂС‚РѕРІ
    pinMode(pwmPin, OUTPUT);
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
}

void Motor::setForward() { // СѓСЃС‚Р°РЅРѕРІРєР° РїСЂСЏРјРѕР№ РїРѕР»СЏСЂРЅРѕСЃС‚Рё
    direction = 1;
    digitalWrite(pin1, LOW); // РћР‘РЇР—РђРўР•Р›Р¬РќРћ РЎРќРђР§РђР›Рђ Р’Р«РљР›Р®Р§Р•РќР�Р• РґР»СЏ РїСЂРµРґРѕС‚РІСЂР°С‰РµРЅРёРµ РїРѕСЂС‡Рё РґСЂР°Р№РІРµСЂР°
    digitalWrite(pin2, HIGH);
}

void Motor::setReverse() { // СѓСЃС‚Р°РЅРѕРІРєР° РѕР±СЂР°С‚РЅРѕР№ РїРѕР»СЏСЂРЅРѕСЃС‚Рё
    direction = -1;
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, HIGH);
}

void Motor::stop() { // РІС‹РєР»СЋС‡РµРЅРёРµ РЁР�Рњ Рё РїРѕРґР°С‡Рё СѓРїСЂР°РІР»СЏСЋС‰РµРіРѕ РЅР°РїСЂСЏР¶РµРЅРёСЏ
    direction = 0;
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    setSpeed(0);
}

void Motor::setSpeed(int speed) { // СѓС‚СЃР°РЅРѕРІРєР° РґР»РёРЅС‹ РёРјРїСѓР»СЊСЃР° РЁР�Рњ
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

void DriveSystem::setSpeed(int leftSpeed, int rightSpeed) { // СѓС‚СЃР°РЅРѕРІРєР° СЃРєРѕСЂРѕСЃС‚Рё
    if (leftSpeed == 0) { // РµСЃР»Рё РїРµСЂРµРґР°РЅ 0 РЅР° Р»РµРІС‹Р№ РґРІРёРіР°С‚РµР»СЊ
        leftMotor->stop(); // РѕСЃС‚Р°РЅРѕРІРєР°
    } else if (leftSpeed < 0) { // РµСЃР»Рё РїРµСЂРµРґР°РЅРѕ Р·РЅР°С‡РµРЅРёРµ РјРµРЅСЊС€Рµ РЅСѓР»СЏ
        leftMotor->setReverse(); // РїРµСЂРµРїРѕР»СЋСЃРѕРІРєР° РґРІРёРіР°С‚РµР»СЏ РЅР° СЂРµРІРµСЂСЃ
        leftMotor->setSpeed(-leftSpeed); // СѓСЃС‚Р°РЅРѕРІРєР° РЁР�Рњ РЅР° Р·Р°РґР°РЅРЅСѓСЋ РґР»РёРЅСѓ РёРјРїСѓР»СЊСЃР°
    } else {
        leftMotor->setForward(); // РїСЂСЏРјР°СЏ РїРѕР»СЋСЃРѕРІРєР° РґРІРёРіР°С‚РµР»СЏ
        leftMotor->setSpeed(leftSpeed); // СѓСЃС‚Р°РЅРѕРІРєР° РЁР�Рњ РЅР° Р·Р°РґР°РЅРЅСѓСЋ РґР»РёРЅСѓ РёРјРїСѓР»СЊСЃР°
    }
    if (rightSpeed == 0) { // Р°РЅР°Р»РѕРіРёС‡РЅРѕ РґР»СЏ РїСЂР°РІРѕРіРѕ РјРѕС‚РѕСЂР°
        rightMotor->stop();
    } else if (rightSpeed < 0) {
        rightMotor->setReverse();
        rightMotor->setSpeed(-rightSpeed);
    } else {
        rightMotor->setForward();
        rightMotor->setSpeed(rightSpeed);
    }
    // СЃРѕС…СЂР°РЅРµРЅРёРµ РїРµСЂРµРґР°РЅРЅС‹С… СЃРєРѕСЂРѕСЃС‚РµР№
    this->leftSpeed = leftSpeed;
    this->rightSpeed = rightSpeed;
}

