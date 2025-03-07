#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ESP32Encoder.h>
#include <Motor.h>
#include <ConfigPin.h>
#include <OmniBase.h>
#include <FIR.h>
#include <MiniPID.h>


float velFL, velFR, velBL, velBR;
float FirVelFL, FirVelFR, FirVelBL, FirVelBR;
float omegaFL, omegaFR, omegaBL, omegaBR;
float FirOmegaFL, FirOmegaFR, FirOmegaBL, FirOmegaBR;
float target_fl, target_fr, target_bl, target_br;


/* ====================================== SETUP FOR OMNIBASE ========================================== */
/* ===================================================================================================== */

// Define encoders and motors
ESP32Encoder enc_FL;
ESP32Encoder enc_FR;
ESP32Encoder enc_BL;
ESP32Encoder enc_BR;

Motor motorFR(MTR1_EN, MTR1_LPWM, MTR1_RPWM);
Motor motorFL(MTR2_EN, MTR2_LPWM, MTR2_RPWM);
Motor motorBR(MTR3_EN, MTR3_LPWM, MTR3_RPWM);
Motor motorBL(MTR4_EN, MTR4_LPWM, MTR4_RPWM);

CalculateEncoder calc_FL(0.1, 200);
CalculateEncoder calc_FR(0.1, 200);
CalculateEncoder calc_BL(0.1, 200);
CalculateEncoder calc_BR(0.1, 200);

// Create an instance of OmniBase
OmniBase omniBase(enc_FL, enc_FR, enc_BL, enc_BR, calc_FL, calc_FR, calc_BL, calc_BR, motorFL, motorFR, motorBL, motorBR);

// Definisi koefisien FIR (hasil desain dari MATLAB)
float firCoefficients[] = {0.0082, 0.0410, 0.1196, 0.2107, 0.2605, 0.2107, 0.1196, 0.0410, 0.0082};
int firOrder = 8; // Orde FIR (jumlah koefisien - 1)

// Inisialisasi objek FIRFilter
FIR firFL(firOrder, firCoefficients);
FIR firFR(firOrder, firCoefficients);
FIR firBL(firOrder, firCoefficients);
FIR firBR(firOrder, firCoefficients);

// Variabel untuk interaktif
float setpoint = 10.2; // Setpoint awal
float kp = 0.005; // Kp awal
float ki = 0.01; // Ki awal
float kd = 0.0; // Kd awal

MiniPID pidFL(kp, ki, kd);
MiniPID pidFR(kp, ki, kd);
MiniPID pidBL(kp, ki, kd);
MiniPID pidBR(kp, ki, kd);

// Variabel Interval dalam ms untuk setiap task
const int updateMotorVelocityInterval = 10; // Interval untuk updateMotorVelocity dalam ms
const int printInterval = 200; // Interval untuk print dalam ms


void updateMotorVelocity(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
      // Calculate velocities
      omniBase.calculate(velFL, velFR, velBL, velBR, omegaFL, omegaFR, omegaBL, omegaBR);

      // Filter using FIR
      // FirOmegaFL = firFL.process(omegaFL);
      // FirOmegaFR = firFR.process(omegaFR);
      // FirOmegaBL = firBL.process(omegaBL);
      // FirOmegaBR = firBR.process(omegaBR);

      FirVelBL = firBL.process(velBL);
      FirVelBR = firBR.process(velBR);
      FirVelFL = firFL.process(velFL);
      FirVelFR = firFR.process(velFR);

      // Update motor speeds using PID
      // omniBase.setMotorSpeeds(
      //     pidFL.getOutput(FirOmegaFL, target_fl),
      //     pidFR.getOutput(FirOmegaFR, target_fr),
      //     pidBL.getOutput(FirOmegaBL, target_bl),
      //     pidBR.getOutput(FirOmegaBR, target_br)
      // );
      omniBase.setMotorSpeeds(
          pidFL.getOutput(FirVelFL, target_fl),
          pidFR.getOutput(FirVelFR, target_fr),
          pidBL.getOutput(FirVelBL, target_bl),
          pidBR.getOutput(FirVelBR, target_br)
      );

      // Maintain a consistent sampling interval
      vTaskDelayUntil(&lastWakeTime, updateMotorVelocityInterval / portTICK_PERIOD_MS);
  }
}

void printTask(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (true) {
    Serial.print("PPR: ");
    Serial.print(enc_FL.getCount());
    Serial.print(" | ");
    Serial.print(enc_FR.getCount());
    Serial.print(" | ");
    Serial.print(enc_BL.getCount());
    Serial.print(" | ");
    Serial.println(enc_BR.getCount());

    vTaskDelayUntil(&lastWakeTime, printInterval / portTICK_PERIOD_MS);
  }
}

/* ====================================== END SETUP FOR OMNIBASE ====================================== */ 
/* ===================================================================================================== */





void setup() {

  /* ===================================================================================================== */
  /* ====================================== GENERAL SETUP =============================================== */

  Serial.begin(115200);

  /* ====================================== END GENERAL SETUP =========================================== */
  /* ===================================================================================================== */


  /* ===================================================================================================== */
  /* ====================================== SETUP FOR OMNIBASE ========================================== */
  // Initialize OmniBase
  omniBase.setup(ENCA_1, ENCB_1, ENCA_2, ENCB_2, ENCA_3, ENCB_3, ENCA_4, ENCB_4);

  // Set PID output limits
  pidFL.setOutputLimits(-1, 1);
  pidFR.setOutputLimits(-1, 1);
  pidBL.setOutputLimits(-1, 1);
  pidBR.setOutputLimits(-1, 1);

  // Create tasks
  xTaskCreate(updateMotorVelocity, "Update Motor Velocity", 2048, NULL, 0, NULL);
  xTaskCreate(printTask, "Print Task", 4096, NULL, 1, NULL);

  /* ====================================== END SETUP FOR OMNIBASE ====================================== */
  /* ===================================================================================================== */

}

void loop() {

}
