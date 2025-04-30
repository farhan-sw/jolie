
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ESP32Encoder.h>
#include <Motor.h>
#include <ConfigPin.h>
#include <OmniBase.h>
#include <FIR.h>
#include <stdio.h>
#include <string.h>


// ==========================================
// ==========================================
// ============== DEFINISI ===================
// ****** TO CHOOSE MODE *******
bool isDebug = false; // false: Mode ROS (tanpa Serial.print), true: Mode Debug (dengan Serial.print)
bool sendInRad = true; // true: kirim nilai omega (rad/s), false: kirim kecepatan linear (m/s)

// ****** TO DEBUG MODE *******
// #define DEBUG_WITH_LED
// #define DEBUG_ERROR
#define MicroROS
// #define MicroROS_DEBUG
#define MicroROS_DEBUG_TUNING
// #define RTOS_DEBUG_RESOURCES

// ****** TO CHOOSE CONTROL ALGORITHM *******
#define UsePID
// #define UseADRC
// ==========================================
// ==========================================

//=========================================
// Konfigurasi Global & Mode Debug
//=========================================
const float wheelRadius = 0.05; // Faktor konversi, misalnya 5 cm

//=========================================
// Setup OmniBase, PID, dan FIR Filter
//=========================================
ESP32Encoder enc_FL;
ESP32Encoder enc_FR;
ESP32Encoder enc_BL;
ESP32Encoder enc_BR;

Motor motorFR(MTR1_EN, MTR1_LPWM, MTR1_RPWM);
Motor motorFL(MTR2_EN, MTR2_LPWM, MTR2_RPWM);
Motor motorBR(MTR3_EN, MTR3_LPWM, MTR3_RPWM);
Motor motorBL(MTR4_EN, MTR4_LPWM, MTR4_RPWM);

CalculateEncoder calc_FL(0.056, 213);
CalculateEncoder calc_FR(0.056, 213);
CalculateEncoder calc_BL(0.056, 213);
CalculateEncoder calc_BR(0.056, 213);

OmniBase omniBase(enc_FL, enc_FR, enc_BL, enc_BR,
                  calc_FL, calc_FR, calc_BL, calc_BR,
                  motorFL, motorFR, motorBL, motorBR);

// Koefisien FIR (hasil desain MATLAB)
float firCoefficients[] = {0.0082, 0.0410, 0.1196, 0.2107, 0.2605, 0.2107, 0.1196, 0.0410, 0.0082};
int firOrder = 8;
FIR firFL(firOrder, firCoefficients);
FIR firFR(firOrder, firCoefficients);
FIR firBL(firOrder, firCoefficients);
FIR firBR(firOrder, firCoefficients);

#ifdef UsePID
#include <MiniPID.h>
// Konstanta PID (tuning lebih lanjut diperlukan)
float kp = 0.005, ki = 0.001, kd = 0.0;
MiniPID pidFL(kp, ki, kd);
MiniPID pidFR(kp, ki, kd);
MiniPID pidBL(kp, ki, kd);
MiniPID pidBR(kp, ki, kd);
#endif

#ifdef UseADRC
#include <ADRC.h>
ADRC adrc_fl;
ADRC adrc_fr;
ADRC adrc_bl;
ADRC adrc_br;
#endif

// Gunakan LED hanya pada mode debug (untuk menghindari konflik di ROS mode)
#ifdef DEBUG_WITH_LED
  #ifndef DEBUG_LED_PIN
    #ifdef LED_BUILTIN
      #define DEBUG_LED_PIN LED_BUILTIN
    #else
      #define DEBUG_LED_PIN 13
    #endif
  #endif
#endif

//=========================================
// Definisi Topic untuk ROS
//=========================================
#define BASE_PUB_FRONT_LEFT_TOPIC "velocity_fl"
#define BASE_PUB_FRONT_RIGHT_TOPIC "velocity_fr"
#define BASE_PUB_BACK_LEFT_TOPIC "velocity_bl"
#define BASE_PUB_BACK_RIGHT_TOPIC "velocity_br"

#define BASE_SUB_FRONT_LEFT_TOPIC "target_velocity_fl"
#define BASE_SUB_FRONT_RIGHT_TOPIC "target_velocity_fr"
#define BASE_SUB_BACK_LEFT_TOPIC "target_velocity_bl"
#define BASE_SUB_BACK_RIGHT_TOPIC "target_velocity_br"

#define TUNING_TOPIC "tuning_topic"

//=========================================
// Variabel Global untuk Komunikasi ROS
//=========================================
#ifdef MicroROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

rcl_publisher_t publisher_fl, publisher_fr, publisher_bl, publisher_br;
std_msgs__msg__Float32 msg_fl, msg_fr, msg_bl, msg_br;
rcl_subscription_t subscriber_fl, subscriber_fr, subscriber_bl, subscriber_br;
std_msgs__msg__Float32 target_msg_fl, target_msg_fr, target_msg_bl, target_msg_br;

rcl_subscription_t tuning_subscriber;
std_msgs__msg__String tuning_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
#endif

//=========================================
// Variabel untuk Feedback dan Pengolahan
//=========================================
float velFL, velFR, velBL, velBR;
float omegaFL, omegaFR, omegaBL, omegaBR;

// Hasil FIR filter
float FirVelFL, FirVelFR, FirVelBL, FirVelBR;
float FirOmegaFL, FirOmegaFR, FirOmegaBL, FirOmegaBR;

// Global target kecepatan dengan proteksi mutex
float target_fl = 0.0f, target_fr = 0.0f, target_bl = 0.0f, target_br = 0.0f;
SemaphoreHandle_t targetMutex = NULL;


//=========================================
// Helper Macro untuk Error Checking (ROS mode tanpa Serial Print)
//=========================================
#ifdef MicroROS
uint32_t rclPublishErrorCount = 0;
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK){ \
    error_loop(); \
  } \
}
#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = fn; \
  if(temp_rc != RCL_RET_OK){ \
    rclPublishErrorCount++; \
    if(rclPublishErrorCount % 100 == 0){ \
      /* Tidak mencetak output */ \
    } \
  } \
}
#endif


//=========================================
// Error Handler
//=========================================
void error_loop(){
  #ifndef DEBUG_ERROR
    while(1){
      delay(100);
    }
  #else
    Serial.println("Critical error encountered. Halting execution.");
    while(1){
      digitalWrite(DEBUG_LED_PIN, !digitalRead(DEBUG_LED_PIN));
      delay(100);
    }
  #endif
}

//=========================================
// Timer Callback untuk Publikasi
//=========================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  #ifdef MicroROS
    if(timer != NULL){
      if(sendInRad){
        msg_fl.data = FirOmegaFL;
        msg_fr.data = FirOmegaFR;
        msg_bl.data = FirOmegaBL;
        msg_br.data = FirOmegaBR;
      } else {
        msg_fl.data = FirVelFL;
        msg_fr.data = FirVelFR;
        msg_bl.data = FirVelBL;
        msg_br.data = FirVelBR;
      }
      RCSOFTCHECK(rcl_publish(&publisher_fl, &msg_fl, NULL));
      RCSOFTCHECK(rcl_publish(&publisher_fr, &msg_fr, NULL));
      RCSOFTCHECK(rcl_publish(&publisher_bl, &msg_bl, NULL));
      RCSOFTCHECK(rcl_publish(&publisher_br, &msg_br, NULL));
    }
  #else
    Serial.print("Feedback: ");
    if(sendInRad){
      Serial.print(" Omega FL: "); Serial.print(FirOmegaFL, 3);
      Serial.print(" Omega FR: "); Serial.print(FirOmegaFR, 3);
      Serial.print(" Omega BL: "); Serial.print(FirOmegaBL, 3);
      Serial.print(" Omega BR: "); Serial.println(FirOmegaBR, 3);
    } else {
      Serial.print(" Vel FL: "); Serial.print(FirVelFL, 3);
      Serial.print(" Vel FR: "); Serial.print(FirVelFR, 3);
      Serial.print(" Vel BL: "); Serial.print(FirVelBL, 3);
      Serial.print(" Vel BR: "); Serial.println(FirVelBR, 3);
    }
  #endif
}

//=========================================
// Subscription Callback untuk Update Target
//=========================================
#ifdef MicroROS
void target_subscription_callback_fl(const void * msgin) {
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  if(xSemaphoreTake(targetMutex, portMAX_DELAY)==pdTRUE){
    target_fl = incoming->data;
    xSemaphoreGive(targetMutex);
  }
}
void target_subscription_callback_fr(const void * msgin) {
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  if(xSemaphoreTake(targetMutex, portMAX_DELAY)==pdTRUE){
    target_fr = incoming->data;
    xSemaphoreGive(targetMutex);
  }
}
void target_subscription_callback_bl(const void * msgin) {
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  if(xSemaphoreTake(targetMutex, portMAX_DELAY)==pdTRUE){
    target_bl = incoming->data;
    xSemaphoreGive(targetMutex);
  }
}
void target_subscription_callback_br(const void * msgin) {
  const std_msgs__msg__Float32* incoming = (const std_msgs__msg__Float32*) msgin;
  if(xSemaphoreTake(targetMutex, portMAX_DELAY)==pdTRUE){
    target_br = incoming->data;
    xSemaphoreGive(targetMutex);
  }
}

void tuning_subscription_callback(const void * msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*) msgin;
  if(xSemaphoreTake(targetMutex, portMAX_DELAY) == pdTRUE){
    #ifdef MicroROS_DEBUG_TUNING
      char buffer[64];
      char cmd[32] = {0};
      float value = 0.0f;
      
      // 1. Copy data dengan proteksi buffer
      strncpy(buffer, msg->data.data, sizeof(buffer)-1);
      buffer[sizeof(buffer)-1] = '\0';
      
      // 2. Parsing command
      int parsed = sscanf(buffer, "%31s %f", cmd, &value);
      
      // 3. Proses command
      if(parsed == 2) {
        // Command SET_TARGET
        if(strcmp(cmd, "SET_TARGET_FL") == 0) {
          target_fl = value;
        } else if(strcmp(cmd, "SET_TARGET_FR") == 0) {
          target_fr = value;
        } else if(strcmp(cmd, "SET_TARGET_BL") == 0) {
          target_bl = value;
        } else if(strcmp(cmd, "SET_TARGET_BR") == 0) {
          target_br = value;
        }
        // Command SET_KP untuk motor spesifik
        else if(strncmp(cmd, "SET_KP_", 7) == 0) {
          const char* motor = cmd + 7;
          if(strcmp(motor, "FL") == 0) pidFL.setP(value);
          else if(strcmp(motor, "FR") == 0) pidFR.setP(value);
          else if(strcmp(motor, "BL") == 0) pidBL.setP(value);
          else if(strcmp(motor, "BR") == 0) pidBR.setP(value);
        }
        // Command SET_KI untuk motor spesifik
        else if(strncmp(cmd, "SET_KI_", 7) == 0) {
          const char* motor = cmd + 7;
          if(strcmp(motor, "FL") == 0) pidFL.setI(value);
          else if(strcmp(motor, "FR") == 0) pidFR.setI(value);
          else if(strcmp(motor, "BL") == 0) pidBL.setI(value);
          else if(strcmp(motor, "BR") == 0) pidBR.setI(value);
        }
        // Command SET_KD untuk motor spesifik
        else if(strncmp(cmd, "SET_KD_", 7) == 0) {
          const char* motor = cmd + 7;
          if(strcmp(motor, "FL") == 0) pidFL.setD(value);
          else if(strcmp(motor, "FR") == 0) pidFR.setD(value);
          else if(strcmp(motor, "BL") == 0) pidBL.setD(value);
          else if(strcmp(motor, "BR") == 0) pidBR.setD(value);
        }
        // Command SET_KP_ALL untuk semua motor
        else if(strcmp(cmd, "SET_KP_ALL") == 0) {
          pidFL.setP(value);
          pidFR.setP(value);
          pidBL.setP(value);
          pidBR.setP(value);
        }
        // Command SET_KI_ALL untuk semua motor
        else if(strcmp(cmd, "SET_KI_ALL") == 0) {
          pidFL.setI(value);
          pidFR.setI(value);
          pidBL.setI(value);
          pidBR.setI(value);
        }
        // Command SET_KD_ALL untuk semua motor
        else if(strcmp(cmd, "SET_KD_ALL") == 0) {
          pidFL.setD(value);
          pidFR.setD(value);
          pidBL.setD(value);
          pidBR.setD(value);
        }
      }
      // Command tanpa parameter
      else if(parsed == 1) {
        #ifdef DEBUG_WITH_LED
          if(strcmp(cmd, "LED_ON") == 0) digitalWrite(DEBUG_LED_PIN, HIGH);
          else if(strcmp(cmd, "LED_OFF") == 0) digitalWrite(DEBUG_LED_PIN, LOW);
        #endif
        
        if(strcmp(cmd, "EMERGENCY_STOP") == 0) {
          target_fl = target_fr = target_bl = target_br = 0.0f;
          omniBase.setMotorSpeeds(0, 0, 0, 0);
        }
      }
    #endif
    xSemaphoreGive(targetMutex);
  }
}

#endif

// Interval update motor: 10 ms
const int updateMotorVelocityInterval = 10;

//=========================================
// Task Update Motor Velocity (Pinned ke Core 1, Stack 4096)
//=========================================
void updateMotorVelocity(void *parameter) {
  (void) parameter;
  TickType_t lastWakeTime = xTaskGetTickCount();
  float local_target_fl, local_target_fr, local_target_bl, local_target_br;
  
  while(true) {
    // Hitung kecepatan linear dan omega (m/s dan rad/s)
    omniBase.calculate(velFL, velFR, velBL, velBR, omegaFL, omegaFR, omegaBL, omegaBR);
    
    // Terapkan FIR filter sesuai pilihan unit
    if(sendInRad) {
      FirOmegaFL = firFL.process(omegaFL);
      FirOmegaFR = firFR.process(omegaFR);
      FirOmegaBL = firBL.process(omegaBL);
      FirOmegaBR = firBR.process(omegaBR);
    } else {
      FirVelFL = firFL.process(velFL);
      FirVelFR = firFR.process(velFR);
      FirVelBL = firBL.process(velBL);
      FirVelBR = firBR.process(velBR);
    }
    
    // Ambil target dengan proteksi mutex
    if(xSemaphoreTake(targetMutex, portMAX_DELAY)==pdTRUE){
      local_target_fl = target_fl;
      local_target_fr = target_fr;
      local_target_bl = target_bl;
      local_target_br = target_br;
      xSemaphoreGive(targetMutex);
    }

    #ifdef UsePID
      // Hitung output PID
      float out_fl, out_fr, out_bl, out_br;
      if(sendInRad) {
        out_fl = pidFL.getOutput(FirOmegaFL, local_target_fl);
        out_fr = pidFR.getOutput(FirOmegaFR, local_target_fr);
        out_bl = pidBL.getOutput(FirOmegaBL, local_target_bl);
        out_br = pidBR.getOutput(FirOmegaBR, local_target_br);
      } else {
        out_fl = pidFL.getOutput(FirVelFL, local_target_fl);
        out_fr = pidFR.getOutput(FirVelFR, local_target_fr);
        out_bl = pidBL.getOutput(FirVelBL, local_target_bl);
        out_br = pidBR.getOutput(FirVelBR, local_target_br);
      }

      omniBase.setMotorSpeeds(out_fl, out_fr, out_bl, out_br);
    #endif

    #ifdef UseADRC
      // Hitung Output ADRC
      float out_fl_adrc, out_fr_adrc, out_bl_adrc, out_br_adrc;
      if(sendInRad){
        out_fl_adrc = computeControlSignal(&adrc_fl, local_target_fl, FirOmegaFL, &out_fl_adrc);
        out_fr_adrc = computeControlSignal(&adrc_fr, local_target_fr, FirOmegaFR, &out_fr_adrc);
        out_bl_adrc = computeControlSignal(&adrc_bl, local_target_bl, FirOmegaBL, &out_bl_adrc);
        out_br_adrc = computeControlSignal(&adrc_br, local_target_br, FirOmegaBR, &out_br_adrc);
      } else {
        out_fl_adrc = computeControlSignal(&adrc_fl, local_target_fl, FirVelFL, &out_fl_adrc);
        out_fr_adrc = computeControlSignal(&adrc_fr, local_target_fr, FirVelFR, &out_fr_adrc);
        out_bl_adrc = computeControlSignal(&adrc_bl, local_target_bl, FirVelBL, &out_bl_adrc);
        out_br_adrc = computeControlSignal(&adrc_br, local_target_br, FirVelBR, &out_br_adrc);
      } 

      omniBase.setMotorSpeeds(out_fl_adrc, out_fr_adrc, out_bl_adrc, out_br_adrc);
    #endif   
    
    vTaskDelayUntil(&lastWakeTime, updateMotorVelocityInterval / portTICK_PERIOD_MS);
  }
}

//=========================================
// Task ROS Executor (Pinned ke Core 0, Stack 2048, Prioritas 3)
//=========================================
#ifdef MicroROS
void rosExecutorTask(void *parameter) {
  (void) parameter;
  const TickType_t executorDelay = 5;
  while(true) {
    #if !isDebug
      rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      if(spin_ret != RCL_RET_OK){
        // Tanpa output Serial pada ROS mode
      }
    #endif
    vTaskDelay(executorDelay / portTICK_PERIOD_MS);
  }
}
#endif


//=========================================
// Task Monitoring Resource (hanya aktif pada mode debug, Pinned ke Core 0)
//=========================================
#ifdef RTOS_DEBUG_RESOURCES
void monitorResourcesTask(void *parameter) {
  (void) parameter;
  while(true) {
    UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("Free Stack: ");
    Serial.println(freeStack);
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Cek setiap 10 detik
  }
}
#endif

//=========================================
// Setup Fungsi Utama
//=========================================
void setup() {
  #ifdef DEBUG_WITH_LED
    Serial.begin(115200);
    pinMode(DEBUG_LED_PIN, OUTPUT);
    digitalWrite(DEBUG_LED_PIN, LOW);
  #endif
  
  targetMutex = xSemaphoreCreateMutex();
  if(targetMutex == NULL){
    #ifdef DEBUG_ERROR
      Serial.println("Failed to create mutex!");
    #endif
    error_loop();
  }
  
  #ifdef MicroROS
    set_microros_transports();
    delay(2000);
    
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    
    RCCHECK(rclc_publisher_init_default(&publisher_fl, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_PUB_FRONT_LEFT_TOPIC));
    RCCHECK(rclc_publisher_init_default(&publisher_fr, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_PUB_FRONT_RIGHT_TOPIC));
    RCCHECK(rclc_publisher_init_default(&publisher_bl, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_PUB_BACK_LEFT_TOPIC));
    RCCHECK(rclc_publisher_init_default(&publisher_br, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_PUB_BACK_RIGHT_TOPIC));
    
    msg_fl.data = msg_fr.data = msg_bl.data = msg_br.data = 0.0f;
    
    RCCHECK(rclc_subscription_init_default(&subscriber_fl, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_SUB_FRONT_LEFT_TOPIC));
    RCCHECK(rclc_subscription_init_default(&subscriber_fr, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_SUB_FRONT_RIGHT_TOPIC));
    RCCHECK(rclc_subscription_init_default(&subscriber_bl, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_SUB_BACK_LEFT_TOPIC));
    RCCHECK(rclc_subscription_init_default(&subscriber_br, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), BASE_SUB_BACK_RIGHT_TOPIC));
    RCCHECK(rclc_subscription_init_default(&tuning_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TUNING_TOPIC));
    
    target_msg_fl.data = target_msg_fr.data = target_msg_bl.data = target_msg_br.data = 0.0f;

    // Initialize the string message data with appropriate capacity
    tuning_msg.data.capacity = 100; // Allocate space for 100 characters
    tuning_msg.data.size = 0;       // Initially empty string
    tuning_msg.data.data = (char*) malloc(tuning_msg.data.capacity * sizeof(char));
    if (tuning_msg.data.data == NULL) {
      error_loop();
    }
    
    const unsigned int timer_timeout_ms = 20;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout_ms), timer_callback));
    
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fl, &target_msg_fl, target_subscription_callback_fl, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_fr, &target_msg_fr, target_subscription_callback_fr, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_bl, &target_msg_bl, target_subscription_callback_bl, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_br, &target_msg_br, target_subscription_callback_br, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &tuning_subscriber, &tuning_msg, tuning_subscription_callback, ON_NEW_DATA));
  #endif
  
  omniBase.setup(ENCA_1, ENCB_1, ENCA_2, ENCB_2, ENCA_3, ENCB_3, ENCA_4, ENCB_4);

  #ifdef UsePID
  pidFL.setOutputLimits(-1, 1);
  pidFR.setOutputLimits(-1, 1);
  pidBL.setOutputLimits(-1, 1);
  pidBR.setOutputLimits(-1, 1);
  #endif

  #ifdef UseADRC
  // Initialize ADRC
  // initADRC(&motor.adrc, sampling_time, 45.0, 1.0, 1, -220, 220); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1
  initADRC(&adrc_fl, updateMotorVelocityInterval, 45.0, 1.0, 1, -1.0, 1.0); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1
  initADRC(&adrc_fr, updateMotorVelocityInterval, 45.0, 1.0, 1, -1.0, 1.0); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1
  initADRC(&adrc_bl, updateMotorVelocityInterval, 45.0, 1.0, 1, -1.0, 1.0); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1
  initADRC(&adrc_br, updateMotorVelocityInterval, 45.0, 1.0, 1, -1.0, 1.0); // standar 3.0, 0.5, 3  b0 = 55.8 -- 45.0, 1.0, 1
  // Setelah inisialisasi, reset ESO
  resetADRC(&adrc_fl);
  resetADRC(&adrc_fr);
  resetADRC(&adrc_bl);
  resetADRC(&adrc_br);
  #endif
  
  // Buat task update motor, pinned ke Core 1 dengan stack 4096 dan prioritas 1
  xTaskCreatePinnedToCore(updateMotorVelocity, "Update Motor Velocity", 4096, NULL, 1, NULL, 1);

  #ifdef MicroROS
    // Buat task ROS executor, pinned ke Core 0 dengan stack 2048 dan prioritas 3
    xTaskCreatePinnedToCore(rosExecutorTask, "ROS Executor Task", 2048, NULL, 3, NULL, 0);
  #endif
  
  #ifdef RTOS_DEBUG_RESOURCES
    // Buat task monitoring resource, pinned ke Core 0 dengan stack 2048 dan prioritas 2
    xTaskCreatePinnedToCore(monitorResourcesTask, "Monitor Resources", 2048, NULL, 2, NULL, 0);
  #endif
}

void loop() {
  delay(10);
}