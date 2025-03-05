#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ESP32Encoder.h>
#include <Motor.h>
#include <ConfigPin.h>
#include <OmniBase.h>
#include <FIR.h>
#include <MiniPID.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#define base_pub_front_left_topic "velocity_fl"
#define base_pub_front_right_topic "velocity_fr"
#define base_pub_back_left_topic "velocity_bl"
#define base_pub_back_right_topic "velocity_br"
#define base_sub_front_left_topic "target_velocity_fl"
#define base_sub_front_right_topic "target_velocity_fr"
#define base_sub_back_left_topic "target_velocity_bl"
#define base_sub_back_right_topic "target_velocity_br"


float velFL, velFR, velBL, velBR;
float FirVelFL, FirVelFR, FirVelBL, FirVelBR;
float omegaFL, omegaFR, omegaBL, omegaBR;
float FirOmegaFL, FirOmegaFR, FirOmegaBL, FirOmegaBR;

/* ===================================================================================================== */
/* ====================================== SETUP FOR MICRO-ROS ========================================== */

// Publishers for measured velocities
rcl_publisher_t publisher_fl;
rcl_publisher_t publisher_fr;
rcl_publisher_t publisher_bl;
rcl_publisher_t publisher_br;

std_msgs__msg__Float32 msg_fl;
std_msgs__msg__Float32 msg_fr;
std_msgs__msg__Float32 msg_bl;
std_msgs__msg__Float32 msg_br;

// Subscribers for target velocities
rcl_subscription_t subscriber_fl;
rcl_subscription_t subscriber_fr;
rcl_subscription_t subscriber_bl;
rcl_subscription_t subscriber_br;

std_msgs__msg__Float32 target_msg_fl;
std_msgs__msg__Float32 target_msg_fr;
std_msgs__msg__Float32 target_msg_bl;
std_msgs__msg__Float32 target_msg_br;

// Global variables to store target values
float target_fl = 0.0f;
float target_fr = 0.0f;
float target_bl = 0.0f;
float target_br = 0.0f;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#ifdef LED_BUILTIN
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Timer callback: publish measured velocities (here for demonstration we simply increment each value by 0.1)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg_fl.data = FirVelFL;
    msg_fr.data = FirVelFR;
    msg_bl.data = FirVelBL;
    msg_br.data = FirVelBR;

    RCSOFTCHECK(rcl_publish(&publisher_fl, &msg_fl, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_fr, &msg_fr, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_bl, &msg_bl, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_br, &msg_br, NULL));
  }
}

// Subscription callback for front-left target velocity:
// If the received target is nonzero, turn LED on; otherwise, turn it off.
void target_subscription_callback_fl(const void * msgin)
{
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  target_fl = incoming->data;
}

// Other subscription callbacks simply update the global target values
void target_subscription_callback_fr(const void * msgin)
{
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  target_fr = incoming->data;
}

void target_subscription_callback_bl(const void * msgin)
{
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  target_bl = incoming->data;
}

void target_subscription_callback_br(const void * msgin)
{
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  target_br = incoming->data;
}

/* ====================================== END SETUP FOR MICRO-ROS ====================================== */
/* ===================================================================================================== */




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

/* ====================================== END SETUP FOR OMNIBASE ====================================== */ 
/* ===================================================================================================== */





void setup() {

  /* ===================================================================================================== */
  /* ====================================== GENERAL SETUP =============================================== */

  /* ====================================== END GENERAL SETUP =========================================== */
  /* ===================================================================================================== */



  /* ===================================================================================================== */
  /* ====================================== SETUP FOR MICRO-ROS ========================================== */

  // Initialize micro-ROS transports (ensure set_microros_transports() is configured appropriately)
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create publishers for measured velocities on four topics
  RCCHECK(rclc_publisher_init_default(
    &publisher_fl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_pub_front_left_topic));
  RCCHECK(rclc_publisher_init_default(
    &publisher_fr, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_pub_front_right_topic));
  RCCHECK(rclc_publisher_init_default(
    &publisher_bl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_pub_back_left_topic));
  RCCHECK(rclc_publisher_init_default(
    &publisher_br, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_pub_back_right_topic));

  // Initialize measured velocity messages to 0.0f
  msg_fl.data = 0.0f;
  msg_fr.data = 0.0f;
  msg_bl.data = 0.0f;
  msg_br.data = 0.0f;

  // Create subscriptions for target velocities on four topics
  RCCHECK(rclc_subscription_init_default(
    &subscriber_fl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_sub_front_left_topic));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_fr, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_sub_front_right_topic));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_bl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_sub_back_left_topic));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_br, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    base_sub_back_right_topic));

  // Initialize target message placeholders
  target_msg_fl.data = 0.0f;
  target_msg_fr.data = 0.0f;
  target_msg_bl.data = 0.0f;
  target_msg_br.data = 0.0f;

  // Create a timer to publish measured velocities every 100ms
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Create executor with capacity for 1 timer + 4 subscriptions = 5 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fl, &target_msg_fl, target_subscription_callback_fl, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fr, &target_msg_fr, target_subscription_callback_fr, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_bl, &target_msg_bl, target_subscription_callback_bl, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_br, &target_msg_br, target_subscription_callback_br, ON_NEW_DATA));

  /* ====================================== END SETUP FOR MICRO-ROS ====================================== */
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

  /* ====================================== END SETUP FOR OMNIBASE ====================================== */
  /* ===================================================================================================== */

}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
