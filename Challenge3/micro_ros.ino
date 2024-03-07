#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher1, publisher2;
std_msgs__msg__Int32 msg_int;
std_msgs__msg__Float32 msg_float;
rcl_subscription_t subscription;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer1, timer2;
int potentiometer_pin = 36;
int pwm_pin = 15;
int pwm_resolution = 8; // PWM resolution (bits)

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback1(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int raw_value = analogRead(potentiometer_pin);
    msg_int.data = raw_value;
  }
}

void timer_callback2(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish the raw value of the potentiometer
    RCSOFTCHECK(rcl_publish(&publisher1, &msg_int, NULL));

    // Convert the raw value to a voltage in the 0-3.3V range
    float voltage = (float)msg_int.data * (3.3 / 4095); // Assuming ADC resolution of 12 bits (4096 levels)
    msg_float.data = voltage;

    // Publish the voltage
    RCSOFTCHECK(rcl_publish(&publisher2, &msg_float, NULL));
  }
}

void pwm_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg_pwm = (const std_msgs__msg__Float32 *)msgin;
  if (msg_pwm != NULL) {
    // Calculate the duty cycle based on the received PWM value
    int duty_cycle = (int)(msg_pwm->data * (1 << pwm_resolution)); // Transform duty cycle to resolution
    digitalWrite(0, duty_cycle); // Set PWM duty cycle
  }
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_esp32_raw_pot"));

  RCCHECK(rclc_publisher_init_default(
      &publisher2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "micro_ros_esp32_voltage"));

  // Create Timer 1 with 10ms interval
  RCCHECK(rclc_timer_init_default(
      &timer1,
      &support,
      RCL_MS_TO_NS(10),
      timer_callback1));

  // Create Timer 2 with 100ms interval
  RCCHECK(rclc_timer_init_default(
      &timer2,
      &support,
      RCL_MS_TO_NS(100),
      timer_callback2));

  // Configure PWM
  ledcSetup(0, 5000, pwm_resolution); // Channel 0, 5000Hz, specified resolution
  ledcAttachPin(pwm_pin, 0); 

  // Create subscription
  RCCHECK(rclc_subscription_init_default(
      &subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/micro_ros_esp32/pwm_duty_cycle"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator)); 
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &msg_float, &pwm_callback, ON_NEW_DATA));

  msg_int.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
