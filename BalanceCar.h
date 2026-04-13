/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-10-08 09:37
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include "EnableInterrupt.h"
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"

// Forward declarations for variables from other headers
extern char key_flag;

MPU6050 mpu;
KalmanFilter kalmanfilter;

//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;

int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;

// For maneuver distance tracking (separate from speed control)
volatile long encoder_distance_left = 0;
volatile long encoder_distance_right = 0;

double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;

void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void balanceCar()
{
  sei();
  
  // Track encoder counts for distance measurement
  encoder_distance_left += encoder_count_left_a;
  encoder_distance_right += encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  
  // SIMPLIFIED: Direct motor control based on motion_mode and speed settings
  // No balance calculations (3-wheeled vehicle doesn't need balance)
  
  int left_speed = setting_car_speed;
  int right_speed = setting_car_speed;
  
  // Apply turn speed
  left_speed -= setting_turn_speed;   // Left motor reduced for right turn
  right_speed += setting_turn_speed;  // Right motor increased for right turn
  
  // Constrain to PWM limits
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Apply left motor
  if (left_speed < 0)
  {
    digitalWrite(AIN1, 1);
    analogWrite(PWMA_LEFT, -left_speed);
  }
  else
  {
    digitalWrite(AIN1, 0);
    analogWrite(PWMA_LEFT, left_speed);
  }
  
  // Apply right motor
  if (right_speed < 0)
  {
    digitalWrite(BIN1, 1);
    analogWrite(PWMB_RIGHT, -right_speed);
  }
  else
  {
    digitalWrite(BIN1, 0);
    analogWrite(PWMB_RIGHT, right_speed);
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();
  
  // DISABLED: Balance features not needed (3-wheeled vehicle)
  // Wire.begin();
  // mpu.initialize();
  
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
  
  // Motor control timer (simplified balanceCar function - no balance, just motor control)
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
