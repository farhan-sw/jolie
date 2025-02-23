#ifndef OMNIBASE_H
#define OMNIBASE_H

#include <ESP32Encoder.h>
#include <CalculateEncoder.h>
#include "Motor.h"

class OmniBase {
public:
    OmniBase(ESP32Encoder& encFL, ESP32Encoder& encFR, ESP32Encoder& encBL, ESP32Encoder& encBR,
             CalculateEncoder& calc_FL, CalculateEncoder& calc_FR, CalculateEncoder& calc_BL, CalculateEncoder& calc_BR,
             Motor& motorFL, Motor& motorFR, Motor& motorBL, Motor& motorBR);

    void setup(int encFLA, int encFLB, int encFRA, int encFRB, int encBLA, int encBLB, int encBRA, int encBRB);
    void setMotorSpeeds(float speedFL, float speedFR, float speedBL, float speedBR);
    void calculate(float& velFL, float& velFR, float& velBL, float& velBR, float& omegaFL, float& omegaFR, float& omegaBL, float& omegaBR);

private:
    ESP32Encoder& encFL;
    ESP32Encoder& encFR;
    ESP32Encoder& encBL;
    ESP32Encoder& encBR;

    Motor& motorFL;
    Motor& motorFR;
    Motor& motorBL;
    Motor& motorBR;

    CalculateEncoder& calc_FL;
    CalculateEncoder& calc_FR;
    CalculateEncoder& calc_BL;
    CalculateEncoder& calc_BR;

    int PulseEncFL;
    int PulseEncFR;
    int PulseEncBL;
    int PulseEncBR;
};

#endif // OMNIBASE_H