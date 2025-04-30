#include "ADRC.h"

// Pin definitions for Motor 1 only
#define ENCODER_A 32
#define ENCODER_B 33
#define RPWM 25
#define LPWM 26
#define REN  18
#define LEN  19

float setpoint = 200.0;


bool manual_test;
const float sampling_time = 0.1; // 10ms sampling time
const int pulses_per_rev = 190;   // motor kecil 180 Change according to your encoder's specification

// Structure for motor
struct Motor {
    int encoderA, encoderB;  // Encoder pins
    int rpwm, lpwm;          // Motor control pins
    volatile int pulse_count;
    float velocity;
    ADRC adrc;
};

// Define single motor
Motor motor = {ENCODER_A, ENCODER_B, RPWM, LPWM, 0, 0};
unsigned long last_time = 0;

// Encoder Interrupt Service Routine
void IRAM_ATTR encoderISR() {
    motor.pulse_count++;
}

// Function to drive motor with control signal
void setMotorPWM(float u) {
    // Make sure parameters have same type (float)
    if (u > 0) {
        analogWrite(motor.rpwm, (int)min(255.0f, abs(u)));
        analogWrite(motor.lpwm, 0);
    } else {
        analogWrite(motor.rpwm, 0);
        analogWrite(motor.lpwm, (int)min(255.0f, abs(u)));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motor 1 Test with ADRC");
    
    // Set motor pins
    pinMode(motor.rpwm, OUTPUT);
    pinMode(motor.lpwm, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    
    // Enable motor driver
    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);
    
    // Set encoder pins
    pinMode(motor.encoderA, INPUT_PULLUP);
    pinMode(motor.encoderB, INPUT_PULLUP);
    
    // Attach Interrupt
    attachInterrupt(digitalPinToInterrupt(motor.encoderA), encoderISR, RISING);
    
    // Initialize ADRC
    initADRC(&motor.adrc, sampling_time, 45.0, 1.0, 1, -220, 220); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1

    // Setelah inisialisasi, reset ESO
    resetADRC(&motor.adrc);
    
    Serial.println("Setup complete, starting control loop...");

    delay(1000);
}

void loop() {
    unsigned long current_time = millis();
    
    if (current_time - last_time >= 100) {
        // Perhitungan kecepatan
        // motor.velocity = (motor.pulse_count * 60.0) / (pulses_per_rev * 0.1);
        float elapsed = (current_time - last_time) / 1000.0;
        motor.velocity = (motor.pulse_count * 60.0) / (pulses_per_rev * elapsed);
        motor.pulse_count = 0;
        
        // Setpoint
        if (Serial.available()){
          setpoint = Serial.parseFloat();
          float setpoint_2 = Serial.parseFloat();
        }
        //float setpoint = 200;
        
        // Update ESO
        updateEso(&motor.adrc, motor.velocity, motor.adrc.u);
        
        // Debug info
        MatrixXf x_hat = getXHAT(&motor.adrc);
        // Serial.print(" | x_hat[0]: "); Serial.print(x_hat(0, 0));
        // Serial.print(" | x_hat[1]: "); Serial.print(x_hat(1, 0));
        // Serial.print(" | x_hat[2]: "); Serial.print(x_hat(2, 0));
        
        // Compute control signal
        float raw_signal;
        float raw_control = computeControlSignal(&motor.adrc, setpoint, motor.velocity, &raw_signal);
        
        // // Tambahkan filter untuk menghindari perubahan kontrol tiba-tiba
        // static float filtered_control = 0;
        // float alpha = 0.3; // Filter coefficient (0-1), lebih kecil = filter lebih kuat
        // filtered_control = alpha * raw_control + (1-alpha) * filtered_control;
        
        // // Terapkan batasan rate of change untuk mencegah osilasi
        // static float prev_control = 0;
        // float max_change_fine = 0.5; // Batas perubahan maksimum per iterasi saat sudah set point
        // float max_change_coarse = 30.0; // Batas perubahan maksimum per iterasi sebelum set point
        
        // if (fabs(motor.velocity - setpoint) < 10.0){
        //   if (filtered_control - prev_control > max_change_fine)
        //     filtered_control = prev_control + max_change_fine;
        //   else if (filtered_control - prev_control < -max_change_fine)
        //     filtered_control = prev_control - max_change_fine;
        // }
        // else{
        //   if (filtered_control - prev_control > max_change_coarse)
        //     filtered_control = prev_control + max_change_coarse;
        //   else if (filtered_control - prev_control < -max_change_coarse)
        //     filtered_control = prev_control - max_change_coarse;
        // }
        
        // prev_control = filtered_control;
        
        //Apply control signal
        // setMotorPWM(filtered_control);
        setMotorPWM(raw_control);
        
        // Debugging
        Serial.print("Velocity: "); Serial.print(motor.velocity);
        Serial.print(" RPM | Control Signal: "); Serial.print(raw_control);
        // Serial.print(" | Filtered: "); Serial.print(filtered_control);
        Serial.print(" | Setpoint: "); Serial.print(setpoint);
        Serial.print(" | Raw Signal: "); Serial.println(raw_signal);

        
        last_time = current_time;
    }
}