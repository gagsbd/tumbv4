/*
 * @Description: Maneuver system for executing a series of predefined movements
 * @Date: 2026-01-23
 */

#ifndef MANEUVER_H
#define MANEUVER_H

// Forward declaration of RGB class (full definition in Rgb.h)
class RGB;
extern RGB rgb;

// External variables from other headers (enums from mode.h)
enum MOTION_MODE;
enum FUNCTION_MODE;
extern MOTION_MODE motion_mode;
extern FUNCTION_MODE function_mode;
extern int setting_car_speed;
extern int setting_turn_speed;
extern int encoder_left_pulse_num_speed;
extern int encoder_right_pulse_num_speed;
extern volatile long encoder_distance_left;
extern volatile long encoder_distance_right;
void captureYawHeading();
float getYawDeltaFromHeading();

// Maneuver types
enum ManeuverType
{
  MANEUVER_FORWARD,    // Move forward a specified distance
  MANEUVER_BACKWARD,   // Move backward a specified distance
  MANEUVER_TURN_LEFT,  // Turn left
  MANEUVER_TURN_RIGHT, // Turn right
  MANEUVER_WAIT,       // Wait for a specified duration
  MANEUVER_IDLE,       // Idle state
};

// Maneuver structure
struct Maneuver
{
  ManeuverType type;
  int value;  // Distance in mm for forward/backward, degrees for turns, milliseconds for wait
};


// Encoder scale: how many counts per mm (calibrated)
#define ENCODER_SCALE 3.29
// Maximum number of maneuvers
#define MAX_MANEUVERS 20

// Global variables for maneuver system
Maneuver maneuvers[MAX_MANEUVERS];
int maneuver_count = 0;
int current_maneuver_index = 0;
boolean maneuver_sequence_active = false;
boolean maneuver_start_pending = false;
unsigned long maneuver_start_time = 0;
int maneuver_target_encoder_count = 0;
int maneuver_initial_encoder_count = 0;
unsigned long maneuver_debug_print_time = 0;

#define TURN_SPEED_FAST 50
#define TURN_SPEED_SLOW 25
#define TURN_SLOWDOWN_DEGREES 15

// Function to add a maneuver to the sequence
void addManeuver(ManeuverType type, int value)
{
  if (maneuver_count < MAX_MANEUVERS)
  {
    maneuvers[maneuver_count].type = type;
    maneuvers[maneuver_count].value = value;
    maneuver_count++;
  }
}

// Function to clear all maneuvers
void clearManeuvers()
{
  maneuver_count = 0;
  current_maneuver_index = 0;
  maneuver_sequence_active = false;
  maneuver_start_pending = false;
}

void requestManeuverStart()
{
  if (maneuver_count > 0 && !maneuver_sequence_active)
  {
    maneuver_start_pending = true;
    motion_mode = START;
    function_mode = IDLE;
    setting_car_speed = 0;
    setting_turn_speed = 0;
  }
}

// Function to start executing the maneuver sequence
void startManeuverSequence()
{
  if (maneuver_count > 0 && !maneuver_sequence_active)
  {
    maneuver_start_pending = false;
    maneuver_sequence_active = true;
    current_maneuver_index = 0;
    motion_mode = START;
    function_mode = IDLE;
    // Reset encoder distance counters for clean distance measurement
    encoder_distance_left = 0;
    encoder_distance_right = 0;
    Serial.println("SEQ START");
  }
}

// Function to stop the maneuver sequence
void stopManeuverSequence()
{
  maneuver_start_pending = false;
  maneuver_sequence_active = false;
  motion_mode = STOP;
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

static unsigned long last_enc_print = 0;
// Function to execute the next maneuver in the sequence
void executeManeuver()
{
  Serial.print("EXE:");
  Serial.print(maneuver_sequence_active);
  Serial.print(":");
  Serial.println(current_maneuver_index);
  
  if (!maneuver_sequence_active || current_maneuver_index >= maneuver_count)
  {
    if (maneuver_sequence_active)
    {
      maneuver_sequence_active = false;
      motion_mode = STANDBY;
      setting_car_speed = 0;
      setting_turn_speed = 0;
      rgb.lightOff();
      Serial.println("DONE");
    }
    return;
  }

  Maneuver current = maneuvers[current_maneuver_index];

  switch (current.type)
  {
  case MANEUVER_FORWARD:
    {
      static int last_mode = -1;
      if (motion_mode != last_mode)
      {
        Serial.print("MODE:");
        Serial.println(motion_mode);
        last_mode = motion_mode;
      }
    }
    
    if (motion_mode != FORWARD)
    {
      motion_mode = FORWARD;
      setting_car_speed = 100; // Increased speed
      setting_turn_speed = 0;
      captureYawHeading();
      maneuver_initial_encoder_count = (encoder_distance_left + encoder_distance_right) / 2;
      //rgb.flashBlueColorFront();
      rgb.flashBrightPurpleColor();
      Serial.print("FWRD START baseline:");
      Serial.println(maneuver_initial_encoder_count);
    }
    else
    {
      Serial.print("FWRDing");
      int current_distance = (encoder_distance_left + encoder_distance_right) / 2;
      int traveled = current_distance - maneuver_initial_encoder_count;
      
      // Debug every 200ms
      //static unsigned long last_enc_print = 0;
      if (millis() - last_enc_print > 200)
      {
        last_enc_print = millis();
        Serial.print("traveled:");
        Serial.print(traveled);
        Serial.print("/");
        Serial.println(current.value);
      }
      
      // Move until target distance reached (scale value so 500 means 500mm)
      if (traveled >= current.value * ENCODER_SCALE)
      {
        motion_mode = STANDBY;
        setting_car_speed = 0;
        current_maneuver_index++;
        Serial.print("FWRD DONE NEXT:");
        Serial.println(current_maneuver_index);
      }
    }
    break;

  case MANEUVER_BACKWARD:
    if (motion_mode != BACKWARD)
    {
      motion_mode = BACKWARD;
      setting_car_speed = -100; // Increased speed
      setting_turn_speed = 0;
      captureYawHeading();
      maneuver_initial_encoder_count = (encoder_distance_left + encoder_distance_right) / 2;
      rgb.flashBlueColorback();
      Serial.print("BACKWARD target:");
      Serial.println(current.value);
    }
    {
      int current_distance = (encoder_distance_left + encoder_distance_right) / 2;
      int traveled = maneuver_initial_encoder_count - current_distance;
      
      // Move until target distance reached (scale value so 500 means 500mm)
      if (traveled >= current.value * ENCODER_SCALE)
      {
        motion_mode = STANDBY;
        setting_car_speed = 0;
        current_maneuver_index++;
        Serial.print("NEXT:");
        Serial.println(current_maneuver_index);
      }
    }
    break;

  case MANEUVER_TURN_LEFT:
    if (motion_mode != TURNLEFT)
    {
      motion_mode = TURNLEFT;
      setting_car_speed = 0;
      captureYawHeading();
      setting_turn_speed = TURN_SPEED_FAST;
      rgb.flashBlueColorLeft();
    }
    {
      float turn_degrees = fabs(getYawDeltaFromHeading());
      float remaining_degrees = current.value - turn_degrees;

      if (remaining_degrees <= TURN_SLOWDOWN_DEGREES)
      {
        setting_turn_speed = TURN_SPEED_SLOW;
      }
      else
      {
        setting_turn_speed = TURN_SPEED_FAST;
      }

      if (turn_degrees >= current.value)
      {
        motion_mode = STANDBY;
        setting_turn_speed = 0;
        current_maneuver_index++;
      }
    }
    break;

  case MANEUVER_TURN_RIGHT:
    if (motion_mode != TURNRIGHT)
    {
      motion_mode = TURNRIGHT;
      setting_car_speed = 0;
      captureYawHeading();
      setting_turn_speed = -TURN_SPEED_FAST;
      rgb.flashBlueColorRight();
    }
    {
      float turn_degrees = fabs(getYawDeltaFromHeading());
      float remaining_degrees = current.value - turn_degrees;

      if (remaining_degrees <= TURN_SLOWDOWN_DEGREES)
      {
        setting_turn_speed = -TURN_SPEED_SLOW;
      }
      else
      {
        setting_turn_speed = -TURN_SPEED_FAST;
      }

      if (turn_degrees >= current.value)
      {
        motion_mode = STANDBY;
        setting_turn_speed = 0;
        current_maneuver_index++;
      }
    }
    break;

  case MANEUVER_WAIT:
    if (motion_mode != STANDBY)
    {
      motion_mode = STANDBY;
      setting_car_speed = 0;
      setting_turn_speed = 0;
      maneuver_start_time = millis();
    }
    if (millis() - maneuver_start_time >= current.value)
    {
      current_maneuver_index++;
    }
    break;

  case MANEUVER_IDLE:
    motion_mode = STANDBY;
    setting_car_speed = 0;
    setting_turn_speed = 0;
    current_maneuver_index++;
    break;

  default:
    break;
  }
}

#endif
