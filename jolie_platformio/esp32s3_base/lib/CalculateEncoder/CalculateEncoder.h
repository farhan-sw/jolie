#ifndef CALCULATE_ENCODER_H
#define CALCULATE_ENCODER_H

#include <Arduino.h>

class CalculateEncoder {
public:
    CalculateEncoder(float wheelDiameter, int ppr);
    void update(int32_t count);
    float getOmega() const;
    float getVelocity() const;

private:
    float wheelDiameter; // Diameter roda dalam meter
    int ppr;             // Pulses Per Revolution
    int32_t lastCount;   // Count encoder sebelumnya
    float omega;         // Kecepatan sudut dalam rad/s
    float velocity;         // Kecepatan dalam m/s
    unsigned long lastTime; // Waktu sebelumnya dalam milidetik
};

#endif // CALCULATE_ENCODER_H