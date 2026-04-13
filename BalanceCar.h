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

#define ENABLE_BALANCE_TEST 0

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

float yaw_angle_deg = 0.0f;
float yaw_target_deg = 0.0f;
float yaw_rate_offset = 0.0f;
int yaw_turn_correction = 0;
unsigned long yaw_last_update_us = 0;

#define GYRO_Z_LSB_PER_DPS 131.0f
#define YAW_HOLD_KP 3.0f
#define YAW_HOLD_MAX_CORRECTION 30

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

void calibrateYawSensor()
{
  long gz_sum = 0;
  const int sample_count = 200;

  for (int sample_index = 0; sample_index < sample_count; sample_index++)
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gz_sum += gz;
    delay(2);
  }

  yaw_rate_offset = (float)gz_sum / sample_count;
  yaw_angle_deg = 0.0f;
  yaw_target_deg = 0.0f;
  yaw_turn_correction = 0;
  yaw_last_update_us = micros();
}

void captureYawHeading()
{
  yaw_target_deg = yaw_angle_deg;
  yaw_turn_correction = 0;
}

float getYawDeltaFromHeading()
{
  return yaw_angle_deg - yaw_target_deg;
}

void updateYawControl()
{
#if ENABLE_BALANCE_TEST
  return;
#else
  unsigned long current_time_us = micros();

  if (yaw_last_update_us == 0)
  {
    yaw_last_update_us = current_time_us;
    return;
  }

  float delta_time = (current_time_us - yaw_last_update_us) / 1000000.0f;
  yaw_last_update_us = current_time_us;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float yaw_rate_dps = ((float)gz - yaw_rate_offset) / GYRO_Z_LSB_PER_DPS;
  if (yaw_rate_dps > -0.4f && yaw_rate_dps < 0.4f)
  {
    yaw_rate_dps = 0.0f;
  }

  yaw_angle_deg += yaw_rate_dps * delta_time;

  if (motion_mode == FORWARD || motion_mode == BACKWARD)
  {
    float yaw_error = yaw_target_deg - yaw_angle_deg;
    yaw_turn_correction = constrain((int)(yaw_error * YAW_HOLD_KP), -YAW_HOLD_MAX_CORRECTION, YAW_HOLD_MAX_CORRECTION);
  }
  else
  {
    yaw_turn_correction = 0;
  }
#endif
}

void balanceCar()
{
  sei();

#if ENABLE_BALANCE_TEST
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;

  encoder_distance_left += encoder_count_left_a;
  encoder_distance_right += encoder_count_right_a;

  encoder_count_left_a = 0;
  encoder_count_right_a = 0;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;

  float yaw_rate_dps = ((float)gz - yaw_rate_offset) / GYRO_Z_LSB_PER_DPS;
  if (yaw_rate_dps > -0.4f && yaw_rate_dps < 0.4f)
  {
    yaw_rate_dps = 0.0f;
  }
  yaw_angle_deg += yaw_rate_dps * dt;

  if (motion_mode == FORWARD || motion_mode == BACKWARD)
  {
    float yaw_error = yaw_target_deg - yaw_angle_deg;
    yaw_turn_correction = constrain((int)(yaw_error * YAW_HOLD_KP), -YAW_HOLD_MAX_CORRECTION, YAW_HOLD_MAX_CORRECTION);
  }
  else
  {
    yaw_turn_correction = 0;
  }

  double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_control_output = setting_turn_speed + yaw_turn_correction + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    motion_mode = STOP;
    carStop();
  }

  if (motion_mode == STOP && key_flag != '4')
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    yaw_turn_correction = 0;
    pwm_left = 0;
    pwm_right = 0;
    carStop();
  }
  else if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    yaw_turn_correction = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left);
    }

    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }
#else
  encoder_distance_left += encoder_count_left_a;
  encoder_distance_right += encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;

  int left_speed = setting_car_speed;
  int right_speed = setting_car_speed;
  int turn_command = setting_turn_speed + yaw_turn_correction;

  left_speed -= turn_command;
  right_speed += turn_command;

  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

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
#endif
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
  
  Wire.begin();
  mpu.initialize();
  calibrateYawSensor();
  
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCountLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
  
  // Motor control timer (simplified balanceCar function - no balance, just motor control)
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
