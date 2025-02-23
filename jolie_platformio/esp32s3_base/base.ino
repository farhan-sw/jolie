#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "../lib/ConfigPin/ConfigPin.h"
#include "../lib/FIR/FIR.h"

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
    msg_fl.data += 0.1f;  // Front Left measured velocity
    msg_fr.data += 0.1f;  // Front Right
    msg_bl.data += 0.1f;  // Back Left
    msg_br.data += 0.1f;  // Back Right

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
  if(target_fl != 0.0f) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
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

void setup() {
  // Initialize micro-ROS transports (ensure set_microros_transports() is configured appropriately)
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create publishers for measured velocities on four topics
  RCCHECK(rclc_publisher_init_default(
    &publisher_fl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocity_fl"));
  RCCHECK(rclc_publisher_init_default(
    &publisher_fr, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocity_fr"));
  RCCHECK(rclc_publisher_init_default(
    &publisher_bl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocity_bl"));
  RCCHECK(rclc_publisher_init_default(
    &publisher_br, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "velocity_br"));

  // Initialize measured velocity messages to 0.0f
  msg_fl.data = 0.0f;
  msg_fr.data = 0.0f;
  msg_bl.data = 0.0f;
  msg_br.data = 0.0f;

  // Create subscriptions for target velocities on four topics
  RCCHECK(rclc_subscription_init_default(
    &subscriber_fl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity_fl"));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_fr, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity_fr"));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_bl, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity_bl"));
  RCCHECK(rclc_subscription_init_default(
    &subscriber_br, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_velocity_br"));

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
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
