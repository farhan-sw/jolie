#ifndef OMNI4WHEEL_H
#define OMNI4WHEEL_H

#include <cmath>
#include <array>

class Omni4Wheel {
private:
    unsigned long* millisPtr; // Pointer to millis() function for timer
    float theta; // Configuration angle in degrees
    float d; // D from the center movement to the wheel
    float diameter; // Diameter of the wheel

    float vx; // Linear velocity in x-axis
    float vy; // Linear velocity in y-axis
    float omega; // Angular velocity

    float FL_speed; // Front left motor speed
    float FR_speed; // Front right motor speed
    float BL_speed; // Back left motor speed
    float BR_speed; // Back right motor speed

    float cos_theta;
    float sin_theta;

    float xPose;
    float yPose;
    float thetaPose;
    float PPR;
    float PPR_xDir;
    float PPR_yDir;

    int prevPulseFL;
    int prevPulseFR;
    int prevPulsesX;
    int prevPulsesY;

public:
    // Constructor
    /**
     * @brief Construct a new Omni4Wheel object
     * @param millisPtr Pointer to millis() function for timer (ms)
     * @param d D from the center movement to the wheel (m)
     * @param diameter Diameter of the wheel (m)
     * @param theta Configuration angle in degrees
    */
    Omni4Wheel(unsigned long* millisPtr, float d, float diameter, float theta = 45.0);

    // reset Pose
    void updatePose(float xPose, float yPose, float thetaPose);

    // Observe encoder pulse and update pose
    void observePose(int pulseFL, int pulseFR, float yawAngle);
    void observePoseExternalEnc(int pulseX, int pulseY);

    // reset acuan pulse
    void resetPulses(int pulseFL, int pulseFR);

    // get Pose
    float getXPose();
    float getYPose();
    float getThetaPose();

    // Set linear velocity in x-axis
    void setVx(float vx);

    // Set linear velocity in y-axis
    void setVy(float vy);

    // Set angular velocity
    void setOmega(float omega);

    // Get Vx Vy Omega
    float getVx();
    float getVy();
    float getOmega();

    // Calculate inverse kinematics
    void InverseCalc();

    // Update New Theta
    void updateTheta(float theta);

    // Get target speed of FL motor
    float getFLSpeed();

    // Get target speed of FR motor
    float getFRSpeed();

    // Get target speed of BL motor
    float getBLSpeed();

    // Get target speed of BR motor
    float getBRSpeed();

    // Fungsi konversi dari m/s ke rotation/s
    float ConvertMStoRPS(float speed_ms, float diameter);

    float getFLSpeedRPS();

    float getFRSpeedRPS();

    float getBLSpeedRPS();

    float getBRSpeedRPS();
};

#endif // OMNI4WHEEL_H
