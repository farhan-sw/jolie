#include "CalculateEncoder.h"
#include <Arduino.h>

CalculateEncoder::CalculateEncoder(float wheelDiameter, int ppr)
    : wheelDiameter(wheelDiameter), ppr(ppr), lastCount(0), omega(0), velocity(0), lastTime(0) {}

void CalculateEncoder::update(int32_t count) {
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastTime;

    if (deltaTime > 0) {
        int32_t deltaCount = count - lastCount;
        float deltaRevolutions = (float)deltaCount / ppr;
        omega = (deltaRevolutions * 2 * PI) / (deltaTime / 1000.0); // rad/s
        velocity = (omega * (wheelDiameter / 2)); // m/s

        lastCount = count;
        lastTime = currentTime;
    }
}

float CalculateEncoder::getOmega() const {
    return omega;
}

float CalculateEncoder::getVelocity() const {
    return velocity;
}