#include "OmniBase.h"

OmniBase::OmniBase(ESP32Encoder& encFL, ESP32Encoder& encFR, ESP32Encoder& encBL, ESP32Encoder& encBR,
                    CalculateEncoder& calc_FL, CalculateEncoder& calc_FR, CalculateEncoder& calc_BL, CalculateEncoder& calc_BR,
                    Motor& motorFL, Motor& motorFR, Motor& motorBL, Motor& motorBR)
    :   encFL(encFL), encFR(encFR), encBL(encBL), encBR(encBR),
        motorFL(motorFL), motorFR(motorFR), motorBL(motorBL), motorBR(motorBR),
        calc_FL(calc_FL), calc_FR(calc_FR), calc_BL(calc_BL), calc_BR(calc_BR)
      {}

void OmniBase::setup(int encFLA, int encFLB, int encFRA, int encFRB, int encBLA, int encBLB, int encBRA, int encBRB) {
    Serial.begin(115200);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    // Attach encoders
    encFL.attachHalfQuad(encFLA, encFLB);
    encFR.attachHalfQuad(encFRA, encFRB);
    encBL.attachHalfQuad(encBLA, encBLB);
    encBR.attachHalfQuad(encBRA, encBRB);

    // Initialize encoder counts
    encFL.setCount(0);
    encFR.setCount(0);
    encBL.setCount(0);
    encBR.setCount(0);
}

void OmniBase::setMotorSpeeds(float speedFL, float speedFR, float speedBL, float speedBR) {
    motorFL.speed(speedFL);
    motorFR.speed(speedFR);
    motorBL.speed(speedBL);
    motorBR.speed(speedBR);
}
void OmniBase::calculate(float& velFL, float& velFR, float& velBL, float& velBR, float& omegaFL, float& omegaFR, float& omegaBL, float& omegaBR){
    PulseEncFL = encFL.getCount();
    PulseEncFR = encFR.getCount();
    PulseEncBL = encBL.getCount();
    PulseEncBR = encBR.getCount();

    calc_FL.update(PulseEncFL);
    calc_FR.update(PulseEncFR);
    calc_BL.update(PulseEncBL);
    calc_BR.update(PulseEncBR);

    omegaFL = calc_FL.getOmega();
    omegaFR = calc_FR.getOmega();
    omegaBL = calc_BL.getOmega();
    omegaBR = calc_BR.getOmega();

    velFL = calc_FL.getVelocity();
    velFR = calc_FR.getVelocity();
    velBL = calc_BL.getVelocity();
    velBR = calc_BR.getVelocity();
}
