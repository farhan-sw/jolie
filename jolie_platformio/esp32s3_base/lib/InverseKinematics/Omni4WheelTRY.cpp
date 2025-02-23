// import math
#include <math.h>

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

public:
    // Constructor
    Omni4Wheel(unsigned long* millisPtr, float d, float diameter, float theta = 45.0) {
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
    }

    // Set linear velocity in x-axis
    void setVx(float vx) {
        this->vx = vx;
    }

    // Set linear velocity in y-axis
    void setVy(float vy) {
        this->vy = vy;
    }

    // Set angular velocity
    void setOmega(float omega) {
        this->omega = omega;
    }

    // Calculate inverse kinematics
    void InverseCalc() {
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

    // Get target speed of FL motor
    float getFLSpeed() {
        return this->FL_speed;
    }

    // Get target speed of FR motor
    float getFRSpeed() {
        return this->FR_speed;
    }

    // Get target speed of BL motor
    float getBLSpeed() {
        return this->BL_speed;
    }

    // Get target speed of BR motor
    float getBRSpeed() {
        return this->BR_speed;
    }

    // Fungsi konversi dari m/s ke rotation/s
    float ConvertMStoRPS(float speed_ms, float diameter) {
        // Menghitung keliling roda
        float circumference = M_PI * diameter;
        
        // Menghitung dan mengembalikan jumlah putaran per detik
        return speed_ms / circumference;
    }

    float getFLSpeedRPS() {
        return ConvertMStoRPS(this->FL_speed, this->diameter);
    }

    float getFRSpeedRPS() {
        return ConvertMStoRPS(this->FR_speed, this->diameter);
    }

    float getBLSpeedRPS() {
        return ConvertMStoRPS(this->BL_speed, this->diameter);
    }

    float getBRSpeedRPS() {
        return ConvertMStoRPS(this->BR_speed, this->diameter);
    }
    
};