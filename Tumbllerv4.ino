/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-12 14:51:36
 * @LastEditTime: 2019-10-11 16:39:57
 * @LastEditors: Please set LastEditors
 */
#include <Arduino.h>
#include "Pins.h"
#include "mode.h"
#include "BalanceCar.h"
#include "Adafruit_NeoPixel.h"
#include "Command.h"
#include "Rgb.h"
#include "Maneuver.h"
#include "Ultrasonic.h"
#include "voltage.h"
#include "EnableInterrupt.h"
#include "MANEUVER_SETUP.h"

// External variable from Maneuver.h
extern boolean maneuver_sequence_active;

unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

void functionMode()
{
  // Execute maneuver sequence when active
  // Button press starts/stops via keyValue() interrupt
  if (maneuver_sequence_active)
  {
    executeManeuver();
  }
}

void setMotionState()
{
  switch (motion_mode)
  {
  case FORWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = 40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case BACKWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = -80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = -40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case TURNLEFT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case BLUETOOTH:
      setting_turn_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = 80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    }
    break;
  case TURNRIGHT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case BLUETOOTH:
      setting_turn_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = -80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    }
    break;
  case STANDBY:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    break;
  case STOP:
    if (millis() - start_prev_time > 1000)
    {
      function_mode = IDLE;
      // DISABLED: Balance angle check (no balance needed for 3-wheeled vehicle)
      // if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      motion_mode = STANDBY;
      rgb.lightOff();
    }
    break;
  case START:
    if (millis() - start_prev_time > 2000)
    {
      // DISABLED: Balance angle check (no balance needed for 3-wheeled vehicle)
      // if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      car_speed_integeral = 0;
      setting_car_speed = 0;
      motion_mode = STANDBY;
      rgb.lightOff();
    }
    break;
  default:
    break;
  }
}

void keyEventHandle()
{
  // DISABLED: Direct key handling disabled
  // Maneuvers are controlled only by button interrupt via keyValue()
}


void setup()
{
  Serial.begin(9600);
  delay(500);
  
  // Initialize only what's needed for maneuver-based control
  rgb.initialize();           // LED feedback
  keyInit();                  // Button to start/stop maneuvers (pin 10)
  carInitialize();            // Initialize motors and encoders (NOT balance)
  initManeuvers();            // Load maneuver sequence
  
  Serial.println("Ready - Press button to start maneuvers");
  start_prev_time = millis();
}
unsigned long print_time = millis();

unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;

void loop()
{
  // Button logic: require 100ms press to start maneuvers
  bool buttonPressed = digitalRead(KEY_MODE) == LOW;
  if (buttonPressed && !buttonWasPressed) {
    buttonPressStart = millis();
    buttonWasPressed = true;
  }
  if (!buttonPressed && buttonWasPressed) {
    buttonWasPressed = false;
    buttonPressStart = 0;
  }
  if (buttonPressed && !maneuver_sequence_active && buttonPressStart && (millis() - buttonPressStart > 100)) {
    startManeuverSequence();
    buttonPressStart = 0;
  }

  if (maneuver_sequence_active)
  {
    executeManeuver();
  }

  rgb.blink(100);

  if (millis() - print_time > 2000)
  {
    print_time = millis();
    Serial.print("Button ready - Maneuvers defined: ");
    Serial.println(maneuver_count);
  }
}
