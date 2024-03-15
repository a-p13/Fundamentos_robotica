#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#define ENCODER_PIN_1 12
#define ENCODER_PIN_2 14
#define MOTOR_PIN_A 33
#define MOTOR_PIN_B 32

#define LED_PIN 12
#define PWM_PIN 16

//VARIABLES
double error;
double e[3] = {0};
int rpm_actual = 0;
int currentPos = 0;
int prev = 0;
float target = 0;
float ref = 0;
int current_clk = 0;
int currentPWM = 0;
int motor_Direction = 0;

rclc_support_t support; //%1
rcl_allocator_t allocator; //%1
rcl_node_t node; //%1

//SUBSCRIBER
rcl_subscription_t subscriber; //%1
std_msgs__msg__Int32 msg_signal; //%1
rclc_executor_t executor_sub; //%1

//PUBLISHER 1
rcl_publisher_t publisher; //%1
std_msgs__msg__Int32 msg; //%1
rclc_executor_t executor_pub; //%1
rcl_timer_t timer;//%1

//PUBLISHER 2
rcl_publisher_t publisher_rpm; //%1
std_msgs__msg__Int32 msg_rpm; //%1
rclc_executor_t executor_pub_rpm;
rcl_timer_t timer_rpm; //%1

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void IRAM_ATTR ISR() {
  if (!digitalRead(ENCODER_PIN_1)) {
    currentPos++;
  }
}

void error_loop(){
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void control(float ref){
  int pwm_controlled;
  error = target - rpm_actual;

  e[1] = e[0];
  e[2] = e[1];
  e[0] = error;

  prev = currentPWM;

  double kp = 0.05;
  double ki = 0.5;
  double kd = 0.001;
  double K1 = (kp + kd/0.02) * e[0];
  double K2 = (-kp + (ki*0.02) - (2*kd/0.02)) * e[1];
  double K3 = (kd/0.02) * e[2];

  pwm_controlled = (prev + K1 + K2 + K3);

  currentPWM = pwm_controlled;

  int e_direction;
  if (error <= -4) {
    e_direction = !motor_Direction;
  } else if (error >= 4) {
    e_direction = motor_Direction;
  }

  ledcWrite(0, currentPWM);
  digitalWrite(MOTOR_PIN_B, e_direction == -1 ? HIGH : LOW);
  digitalWrite(MOTOR_PIN_A, e_direction == 1 ? HIGH : LOW);
  
  }


void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32* msg_signal = (const std_msgs__msg__Float32*)msgin;
  ref = msg_signal->data;
  float pulsos = ref*340;

  if(pulsos < 0){
    pulsos *= -1;
    motor_Direction = -1;
  }
  else{
    motor_Direction = 1;
  }

  target = pulsos;

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    rpm_actual = (current_clk *600)/(2*374);
    msg.data = rpm_actual;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    current_clk = 0;
    
  }
}

void timer_callback_rpm(rcl_timer_t *timer_rpm, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer_rpm != NULL)
  {
    msg_rpm.data = target;
    RCSOFTCHECK(rcl_publish(&publisher_rpm, &msg_rpm, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  //pinMode(PWM_PIN, OUTPUT);
  ledcSetup(0, 5000, 8); // Canal 0, 5000 Hz, 8 bits de resolución
  ledcAttachPin(PWM_PIN, 0); // Asocia PWM_PIN con el canal 0 de PWM
  pinMode(ENCODER_PIN_1, INPUT);
  pinMode(ENCODER_PIN_2, INPUT);
  delay(200);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2), ISR, CHANGE);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Inicializa el nodo 
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Inicializa el suscriptor 
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "signal"));

  // Inicializa el publisher ROS para la señal de control
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "output_ref"));

  // Inicializa el publisher ROS para la señal de control
  RCCHECK(rclc_publisher_init_default(
    &publisher_rpm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "output_motor"));

  const unsigned int timer_timeout = 200;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

   RCCHECK(rclc_timer_init_default(
      &timer_rpm,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback_rpm));

      RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
      RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
      RCCHECK(rclc_executor_add_timer(&executor_pub, &timer_rpm));
    
      RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));
      RCCHECK(rclc_executor_init(&executor_pub_rpm, &support.context, 2, &allocator));
      RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
      RCCHECK(rclc_executor_add_timer(&executor_pub_rpm, &timer_rpm));
      RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
      RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_signal, &subscription_callback, ON_NEW_DATA));

      msg.data = 0;
      msg_rpm.data = 0;

}

void loop() {
  delay(50);
  control(target);
  RCCHECK(rclc_executor_spin_some(&executor_pub_rpm, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
