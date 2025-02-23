#include "Motor.h"

Motor::Motor(int enPin, int lpwmPin, int rpwmPin) 
  : enPin(enPin), lpwmPin(lpwmPin), rpwmPin(rpwmPin) {
  pinMode(enPin, OUTPUT);
  pinMode(lpwmPin, OUTPUT);
  pinMode(rpwmPin, OUTPUT);
  digitalWrite(enPin, HIGH); // Enable motor driver
}

void Motor::speed(float pwm) {
  int pwmValue = abs(pwm) * 255;
  if (pwm > 0) {
    analogWrite(lpwmPin, pwmValue);
    analogWrite(rpwmPin, 0);
  } else {
    analogWrite(lpwmPin, 0);
    analogWrite(rpwmPin, pwmValue);
  }
}