// Include Eigen library
#include "eigen.h"
#include <Eigen/Dense>

using namespace Eigen;

// Define a structure to hold ADRC parameters and states
typedef struct {
    // Input parameters
    float t_sampling;
    float b0;
    float t_settling;
    int k_eso;

    // Protection limits
    float u_min, u_max; // Min/Max control signal

    // Computed parameters
    float s_cl, z_eso;
    float l1, l2, l3;
    float Kp, Kd;

    // Matrices for ESO and control
    MatrixXf Ad, Bd, Cd, Lc, x_hat;

    // Output control signal
    float u, u_prev;
} ADRC;

// Initialize ADRC parameters and matrices
void initADRC(ADRC *adrc, float t_sampling, float b0, float t_settling, int k_eso, float u_min, float u_max) {
    adrc->t_sampling = t_sampling;
    adrc->b0 = b0;
    adrc->t_settling = t_settling;
    adrc->k_eso = k_eso;

    adrc->u_min = u_min;
    adrc->u_max = u_max;

    // ESO tuning
    adrc->s_cl = -6.0 / t_settling;
    adrc->z_eso = exp(k_eso * adrc->s_cl * t_sampling);
    adrc->l1 = 1 - pow(adrc->z_eso, 3);
    adrc->l2 = 3.0f / (2 * t_sampling) * pow((1 - adrc->z_eso), 2) * (1 + adrc->z_eso);
    adrc->l3 = 1.0f / pow(t_sampling, 2) * pow((1 - adrc->z_eso), 3);

    // Control gains (adjusted for stability)
    adrc->Kp = 0.8 * (adrc->s_cl * adrc->s_cl);
    adrc->Kd = -1.2 * (2 * adrc->s_cl);

    // State-space matrices
    adrc->Ad = MatrixXf(3, 3);
    adrc->Ad << 1, t_sampling, pow(t_sampling, 2) / 2.0f,
                0, 1, t_sampling,
                0, 0, 1;

    adrc->Bd = MatrixXf(3, 1);
    adrc->Bd << b0 * pow(t_sampling, 2) / 2.0f, b0 * t_sampling, 0;

    adrc->Cd = MatrixXf(1, 3);
    adrc->Cd << 1, 0, 0;

    adrc->Lc = MatrixXf(3, 1);
    adrc->Lc << adrc->l1, adrc->l2, adrc->l3;

    adrc->x_hat = MatrixXf::Zero(3, 1); // Initialize state to zero
    adrc->u = 0;
    adrc->u_prev = 0;
}

// Reset ADRC states
void resetADRC(ADRC *adrc) {
    adrc->x_hat = MatrixXf::Zero(3, 1);
    adrc->u = 0;
    adrc->u_prev = 0;
}

// Update the Extended State Observer (ESO)
void updateEso(ADRC *adrc, float y, float u) {
    adrc->x_hat = (adrc->Ad - (adrc->Lc * adrc->Cd) * adrc->Ad) * adrc->x_hat
                + (adrc->Bd - (adrc->Lc * adrc->Cd) * adrc->Bd) * u
                + adrc->Lc * y;
}

// Compute the control signal with filtering and rate limit
float computeControlSignal(ADRC *adrc, float setpoint, float velocity, float *raw_u_output) {
    // Raw control calculation
    float raw_u = (adrc->Kp * (setpoint - adrc->x_hat(0, 0)) 
                 - adrc->Kd * adrc->x_hat(1, 0) 
                 - adrc->x_hat(2, 0)) / adrc->b0;

    // Clamp to control limits
    raw_u = fmax(adrc->u_min, fmin(raw_u, adrc->u_max));
    // Serial.println(raw_u);
    *raw_u_output = raw_u;

    // Apply a low-pass filter: u = alpha * u_prev + (1 - alpha) * raw_u
    float alpha = 0.8;  // Adjust between 0 (no filtering) to 1 (strong filtering)
    float filtered_u = alpha * adrc->u_prev + (1 - alpha) * raw_u;

    // Limit the rate of change (slew rate limiter)
    float max_delta_u = 0.05; // Adjust based on system response
    // if (fabs(filtered_u - adrc->u_prev) > max_delta_u) {
    //     if (filtered_u > adrc->u_prev) 
    //         filtered_u = adrc->u_prev + max_delta_u;
    //     else 
    //         filtered_u = adrc->u_prev - max_delta_u;
    // }
    float max_change_coarse = 30;

    if (fabs(velocity - setpoint) < 10.0){
          if (filtered_u - adrc->u_prev > max_delta_u)
            filtered_u = adrc->u_prev + max_delta_u;
          else if (filtered_u - adrc->u_prev < -max_delta_u)
            filtered_u = adrc->u_prev - max_delta_u;
    }
    else{
        if (filtered_u - adrc->u_prev > max_change_coarse)
        filtered_u = adrc->u_prev + max_change_coarse;
        else if (filtered_u - adrc->u_prev < -max_change_coarse)
        filtered_u = adrc->u_prev - max_change_coarse;
    }
    // if (fabs(velocity - setpoint) < 10.0){
    //       if (filtered_u - adrc->u_prev > max_change_fine){
    //         filtered_u = adrc->u_prev + max_change_fine;
    //       }
    //       else if (filtered_u - adrc->u_prev < -max_change_fine){
    //         filtered_u = adrc->u_prev - max_change_fine;
    //       }
    // }else{
    //     if (filtered_u - adrc->u_prev > max_delta_u)
    //     filtered_u = adrc->u_prev + max_delta_u;
    //     else if (filtered_u - adrc->u_prev < -max_delta_u)
    //     filtered_u = adrc->u_prev - max_delta_u;
    // }
    // Update control output
    adrc->u = filtered_u;
    adrc->u_prev = adrc->u;

    return adrc->u;
}

// Get estimated states
MatrixXf getXHAT(ADRC *adrc) {
    return adrc->x_hat;
}