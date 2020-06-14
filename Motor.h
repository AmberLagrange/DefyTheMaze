#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Arduino.h"

class Motor
{
  public:
    static const int SPEED = 180;
    
    Motor(int dir, int pwmPin, int dirPin);
    void forward(bool rotating);
    void reverse(bool rotating);
    void halt();
    
    int getRotSpeed() {return mRotSpeed;}
    void setRotSpeed(int rotSpeed) {mRotSpeed = constrain(rotSpeed, 0, 255);}
  private:
    int mDir;
    int mPwmPin;
    int mDirPin;
    int mRotSpeed;
};

#endif //MOTORCONTROL_H
