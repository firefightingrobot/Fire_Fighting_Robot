#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <ArduinoHardware.h>

// Encoder Pins
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 5

// Motor Control Pins
#define EN_L 6
#define IN1_L 7
#define IN2_L 8
#define EN_R 11
#define IN1_R 9
#define IN2_R 10

// Relay Pins
#define RELAY_RIGHT 13
#define RELAY_LEFT 12

// Wheel Parameters
double wheel_rad = 0.0325, wheel_sep = 0.34; // Wheel radius (m), wheel separation (m)

// Tick Variables
std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;
boolean Direction_left = true;
boolean Direction_right = true;
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// ROS Node Handle
ros::NodeHandle nh;

// Publishers for encoder tick counts
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Subscribers for velocity commands
double speed_ang = 0, speed_lin = 0;
double w_r = 0, w_l = 0;

void messageCb(const geometry_msgs::Twist& msg) {
  double linear_vel = msg.linear.x;
  double angular_vel = msg.angular.z;

  w_l = (linear_vel * 2) - ((angular_vel * wheel_sep) / (2.0 * wheel_rad));
  w_r = (linear_vel * 2) + ((angular_vel * wheel_sep) / (2.0 * wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

// Relay Control for Left Pump
void leftPumpControlCb(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(RELAY_LEFT, HIGH); // Turn ON left pump
  } else {
    digitalWrite(RELAY_LEFT, LOW);  // Turn OFF left pump
  }
}
ros::Subscriber<std_msgs::Bool> leftPumpSub("left_pump_control", &leftPumpControlCb);

// Relay Control for Right Pump
void rightPumpControlCb(const std_msgs::Bool& msg) {
  if (msg.data) {
    digitalWrite(RELAY_RIGHT, HIGH); // Turn ON right pump
  } else {
    digitalWrite(RELAY_RIGHT, LOW);  // Turn OFF right pump
  }
}
ros::Subscriber<std_msgs::Bool> rightPumpSub("right_pump_control", &rightPumpControlCb);

// Timing for Encoder Publishing
const int interval = 100; // 100 ms
long previousMillis = 0;

// Increment Tick Counts
void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);
  Direction_right = (val == HIGH);
  right_wheel_tick_count.data += Direction_right ? 1 : -1;

  // Handle overflow
  if (right_wheel_tick_count.data > encoder_maximum) right_wheel_tick_count.data = encoder_minimum;
  if (right_wheel_tick_count.data < encoder_minimum) right_wheel_tick_count.data = encoder_maximum;
}

void left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);
  Direction_left = (val == HIGH);
  left_wheel_tick_count.data += Direction_left ? 1 : -1;

  // Handle overflow
  if (left_wheel_tick_count.data > encoder_maximum) left_wheel_tick_count.data = encoder_minimum;
  if (left_wheel_tick_count.data < encoder_minimum) left_wheel_tick_count.data = encoder_maximum;
}

// Motor Control Functions
void MotorL(int Pulse_Width1) {
  if (Pulse_Width1 > 0) {
    analogWrite(EN_L, Pulse_Width1);  // Forward
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else if (Pulse_Width1 < 0) {
    Pulse_Width1 = abs(Pulse_Width1);
    analogWrite(EN_L, Pulse_Width1);  // Reverse
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    analogWrite(EN_L, 0);  // Stop
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
  }
}

void MotorR(int Pulse_Width2) {
  if (Pulse_Width2 > 0) {
    analogWrite(EN_R, Pulse_Width2);  // Forward
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  } else if (Pulse_Width2 < 0) {
    Pulse_Width2 = abs(Pulse_Width2);
    analogWrite(EN_R, Pulse_Width2);  // Reverse
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else {
    analogWrite(EN_R, 0);  // Stop
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
  }
}

void setup() {
  // Encoder Setup
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor Setup
  pinMode(EN_L, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);

  // Relay Setup
  pinMode(RELAY_RIGHT, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);
  digitalWrite(RELAY_RIGHT, LOW);
  digitalWrite(RELAY_LEFT, LOW);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(sub);
  nh.subscribe(leftPumpSub);
  nh.subscribe(rightPumpSub);
}

void loop() {
  // Motor Control
  MotorL(w_l * 180);
  MotorR(w_r * 180);

  // Publish Encoder Ticks
  long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    rightPub.publish(&right_wheel_tick_count);
    leftPub.publish(&left_wheel_tick_count);
  }

  // ROS Spin
  nh.spinOnce();
}
