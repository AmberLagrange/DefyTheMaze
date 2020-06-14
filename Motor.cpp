#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int dir, int pwmPin, int dirPin) :
mDir(dir), mPwmPin(pwmPin), mDirPin(dirPin), mRotSpeed(SPEED)
{
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void Motor::forward(bool rotating)
{
  analogWrite(mPwmPin, mRotSpeed);
  digitalWrite(mDirPin, !mDir ? HIGH : LOW);
  setRotSpeed(SPEED * (1 - 0.15 * rotating));
}

void Motor::reverse(bool rotating)
{
  analogWrite(mPwmPin, mRotSpeed);
  digitalWrite(mDirPin, !mDir ? LOW : HIGH);
  setRotSpeed(SPEED * (1 - 0.15 * rotating));
}

void Motor::halt()
{
  digitalWrite(mPwmPin, LOW);
  digitalWrite(mDirPin, LOW);
}

