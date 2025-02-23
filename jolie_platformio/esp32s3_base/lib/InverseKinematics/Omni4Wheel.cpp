#include "Omni4Wheel.h"

// Constructor
Omni4Wheel::Omni4Wheel(unsigned long* millisPtr, float d, float diameter, float theta) {
    this->millisPtr = millisPtr;
    this->theta = theta;

    this->d = d;
    this->diameter = diameter;

    this->vx = 0.0;
    this->vy = 0.0;
    this->omega = 0.0;
    this->FL_speed = 0.0;
    this->FR_speed = 0.0;
    this->BL_speed = 0.0;
    this->BR_speed = 0.0;

    // Hitung cos dan sin berdasarkan theta
    this->cos_theta = cos(theta * M_PI / 180.0);
    this->sin_theta = sin(theta * M_PI / 180.0);

    this->xPose = 0.0f;
    this->yPose = 0.0f;
    this->thetaPose = 0.0f;
    this->PPR = 210;
    this->prevPulseFL = 0;
    this->prevPulseFR = 0;

    this->prevPulsesX = 0;
    this->prevPulsesY = 0;
    this->PPR_xDir = 125 * 4;
    this->PPR_yDir = 360 * 2;
}


void Omni4Wheel::updatePose(float xPose, float yPose, float thetaPose) {
    this->xPose = xPose;
    this->yPose = yPose;
    this->thetaPose = thetaPose;
}

void Omni4Wheel::observePose(int pulseFL, int pulseFR, float yawAngle) {
    // Bang angka 2 itu buatan yakkk, angka tuningan :)
    this->xPose = 3.1415926535 * sqrt(2) * (this->diameter / 2.0f) / this->PPR * ((pulseFR - this->prevPulseFR) - (pulseFL - this->prevPulseFL));
    this->yPose = (-1) * 3.1415926535 * sqrt(2) * (this->diameter / 2.0f) / this->PPR * ((pulseFR - this->prevPulseFR) + (pulseFL - this->prevPulseFL));
    this->thetaPose = yawAngle;
}

void Omni4Wheel::observePoseExternalEnc(int pulseX, int pulseY) {
    this->xPose = ((pulseX - this->prevPulsesX) / this->PPR_xDir) * (2 * 3.1415926535 * 29 / 1000.0f);
    this->yPose = ((pulseY - this->prevPulsesY) / this->PPR_yDir) * (2 * 3.1415926535 * 29 / 1000.0f);
}

void Omni4Wheel::resetPulses(int pulseFL, int pulseFR) {
    this->prevPulseFL = pulseFL;
    this->prevPulseFR = pulseFR;

    this->prevPulsesX = pulseFL;
    this->prevPulsesY = pulseFR;
}

float Omni4Wheel::getXPose() {
    return this->xPose;
}
float Omni4Wheel::getYPose() {
    return this->yPose;
}
float Omni4Wheel::getThetaPose() {
    return this->thetaPose;
}

// Set linear velocity in x-axis
void Omni4Wheel::setVx(float vx) {
    this->vx = vx;
}

// Set linear velocity in y-axis
void Omni4Wheel::setVy(float vy) {
    this->vy = vy;
}

// Set angular velocity
void Omni4Wheel::setOmega(float omega) {
    this->omega = omega;
}

// Calculate inverse kinematics
void Omni4Wheel::InverseCalc() {
    // Calculate motor speeds based on vx, vy, and omega
    // Calculate v(t), vn(t), and omega(t)
    // v(t) = vx * cos(theta) + vy * sin(theta)
    // vn(t) = -vx * sin(theta) + vy * cos(theta)
    // omega(t) = omega
    float v = vx * cos_theta + vy * sin_theta;
    float vn = -vx * sin_theta + vy * cos_theta;

    // Calculate the target speed of each motor
    // FL_speed = vn + omega * d
    // BL_speed = -v + omega * d
    // BR_speed = -vn + omega * d
    // FR_speed = v + omega * d
    this->FL_speed = vn + omega * d;
    this->BL_speed = -v + omega * d;
    this->BR_speed = -vn + omega * d;
    this->FR_speed = v + omega * d;
}

void Omni4Wheel::updateTheta(float theta)
{
    // Hitung cos dan sin berdasarkan theta
    this->theta = theta;
    this->cos_theta = cos(theta * M_PI / 180.0);
    this->sin_theta = sin(theta * M_PI / 180.0);
}

// Get target speed of FL motor
float Omni4Wheel::getFLSpeed() {
    return this->FL_speed;
}

// Get target speed of FR motor
float Omni4Wheel::getFRSpeed() {
    return this->FR_speed;
}

// Get target speed of BL motor
float Omni4Wheel::getBLSpeed() {
    return this->BL_speed;
}

// Get target speed of BR motor
float Omni4Wheel::getBRSpeed() {
    return this->BR_speed;
}

// Fungsi konversi dari m/s ke rotation/s
float Omni4Wheel::ConvertMStoRPS(float speed_ms, float diameter) {
    // Menghitung keliling roda
    float circumference = M_PI * diameter;
    
    // Menghitung dan mengembalikan jumlah putaran per detik
    return speed_ms / circumference;
}

float Omni4Wheel::getFLSpeedRPS() {
    return ConvertMStoRPS(this->FL_speed, this->diameter);
}

float Omni4Wheel::getFRSpeedRPS() {
    return ConvertMStoRPS(this->FR_speed, this->diameter);
}

float Omni4Wheel::getBLSpeedRPS() {
    return ConvertMStoRPS(this->BL_speed, this->diameter);
}

float Omni4Wheel::getBRSpeedRPS() {
    return ConvertMStoRPS(this->BR_speed, this->diameter);
}


float Omni4Wheel::getVx() {
    return this->vx;
}

float Omni4Wheel::getVy() {
    return this->vy;
}   

float Omni4Wheel::getOmega() {
    return this->omega;
}
