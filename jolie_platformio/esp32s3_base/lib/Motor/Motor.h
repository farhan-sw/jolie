#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
  Motor(int enPin, int lpwmPin, int rpwmPin);
  void speed(float pwm);

private:
  int enPin;
  int lpwmPin;
  int rpwmPin;
};

#endif // MOTOR_H