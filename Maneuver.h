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
int maneuver_turn_direction_sign = 0;
boolean maneuver_turn_settling = false;
unsigned long maneuver_turn_settle_start = 0;

#define TURN_SPEED_FAST 50
#define TURN_SPEED_SLOW 25
#define TURN_DIRECTION_DETECT_DEGREES 2.0f
#define TURN_SETTLE_TOLERANCE_DEGREES 4.0f
#define TURN_SPEED_KP 1.2f
#define TURN_SETTLE_MS 120
#define TURN_ANGLE_SCALE 0.98f

// Encoder-based turn calibration (derived from physical dimensions):
// Wheel diameter = 68mm, wheelbase = 170mm, ENCODER_SCALE = 3.29 counts/mm
// Pivot turn arc per degree per wheel = PI * 170 / 360 = 1.484 mm
// Counts per degree = 1.484 * 3.29 = 4.88, adjusted for observed overshoot
#define ENCODER_TURN_SCALE_PER_DEGREE 5.02f
// Settle threshold ~1 degree worth of counts (1 * 4.88 ≈ 5)
#define ENCODER_TURN_SETTLE_COUNTS 5
// KP equivalent to yaw TURN_SPEED_KP (1.2) scaled to encoder counts: 1.2 / 4.88 ≈ 0.25
#define ENCODER_TURN_SPEED_KP 0.25f

// Flag: false = use yaw/gyro for turns, true = use encoder counts for turns
boolean maneuver_use_encoder_turns = false;

// Speed for forward/backward maneuvers (0-255)
int maneuver_drive_speed = 40;

int getTurnSpeedCommand(float remaining_degrees)
{
  int turn_speed = (int)(remaining_degrees * TURN_SPEED_KP);

  if (turn_speed < TURN_SPEED_SLOW)
  {
    turn_speed = TURN_SPEED_SLOW;
  }

  if (turn_speed > TURN_SPEED_FAST)
  {
    turn_speed = TURN_SPEED_FAST;
  }

  return turn_speed;
}

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
  maneuver_turn_direction_sign = 0;
  maneuver_turn_settling = false;
  maneuver_turn_settle_start = 0;
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
    Serial.println(F("SEQ START"));
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
  maneuver_turn_direction_sign = 0;
  maneuver_turn_settling = false;
  maneuver_turn_settle_start = 0;
}

static unsigned long last_enc_print = 0;
// Function to execute the next maneuver in the sequence
void executeManeuver()
{
  if (!maneuver_sequence_active || current_maneuver_index >= maneuver_count)
  {
    if (maneuver_sequence_active)
    {
      maneuver_sequence_active = false;
      motion_mode = STANDBY;
      setting_car_speed = 0;
      setting_turn_speed = 0;
      rgb.lightOff();
      Serial.println(F("DONE"));
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
        Serial.print(F("MODE:"));
        Serial.println(motion_mode);
        last_mode = motion_mode;
      }
    }
    
    if (motion_mode != FORWARD)
    {
      motion_mode = FORWARD;
      setting_car_speed = maneuver_drive_speed;
      setting_turn_speed = 0;
      captureYawHeading();
      maneuver_initial_encoder_count = ((encoder_distance_left + encoder_distance_right)-245) / 2;
      //rgb.flashBlueColorFront();
      rgb.flashBrightPurpleColor();
      Serial.print(F("FWRD START baseline:"));
      Serial.println(maneuver_initial_encoder_count);
      Serial.print(F("Start:"));
      Serial.print(millis());
    }
    else
    {
      int current_distance = (encoder_distance_left + encoder_distance_right) / 2;
      int traveled = current_distance - maneuver_initial_encoder_count;
      
      // Debug every 200ms
      //static unsigned long last_enc_print = 0;
      if (millis() - last_enc_print > 200)
      {
        last_enc_print = millis();
        Serial.print(F("traveled:"));
        Serial.print(traveled);
        Serial.print(F("/"));
        Serial.println(current.value);
      }
      
      // Move until target distance reached (scale value so 500 means 500mm)
      if (traveled >= current.value * ENCODER_SCALE)
      {
        motion_mode = STANDBY;
        setting_car_speed = 0;
        current_maneuver_index++;
        Serial.print(F("FWRD DONE NEXT:"));
        Serial.println(current_maneuver_index);

        Serial.print(F("End:"));
      Serial.print(millis());
      }
    }
    break;

  case MANEUVER_BACKWARD:
    if (motion_mode != BACKWARD)
    {
      motion_mode = BACKWARD;
      setting_car_speed = -maneuver_drive_speed;
      setting_turn_speed = 0;
      captureYawHeading();
      maneuver_initial_encoder_count = ((encoder_distance_left + encoder_distance_right)-245) / 2;
      rgb.flashBrightPurpleColorBack();
      Serial.print(F("BACK START baseline:"));
      Serial.println(maneuver_initial_encoder_count);
    }
    else
    {
      int current_distance = (encoder_distance_left + encoder_distance_right) / 2;
      int traveled = current_distance - maneuver_initial_encoder_count;

      if (millis() - last_enc_print > 200)
      {
        last_enc_print = millis();
        Serial.print(F("traveled:"));
        Serial.print(traveled);
        Serial.print(F("/"));
        Serial.println(current.value);
      }
      
      // Encoders accumulate counts in both directions, so reverse distance is also measured by count increase from the baseline.
      if (traveled >= current.value * ENCODER_SCALE)
      {
        motion_mode = STANDBY;
        setting_car_speed = 0;
        current_maneuver_index++;
        Serial.print(F("BACK DONE NEXT:"));
        Serial.println(current_maneuver_index);
      }
    }
    break;

  case MANEUVER_TURN_LEFT:
    if (maneuver_use_encoder_turns)
    {
      // --- Encoder-based left turn ---
      if (motion_mode != TURNLEFT && !maneuver_turn_settling)
      {
        motion_mode = TURNLEFT;
        setting_car_speed = 0;
        maneuver_turn_settling = false;
        maneuver_initial_encoder_count = (encoder_distance_left + encoder_distance_right) / 2;
        maneuver_target_encoder_count = (int)(current.value * ENCODER_TURN_SCALE_PER_DEGREE);
        setting_turn_speed = TURN_SPEED_FAST;
        rgb.flashGoldColorLeft();
        Serial.print(F("ENC TLEFT START tgt:"));
        Serial.println(maneuver_target_encoder_count);
      }
      {
        int current_enc = (encoder_distance_left + encoder_distance_right) / 2;
        int traveled = current_enc - maneuver_initial_encoder_count;
        int remaining = maneuver_target_encoder_count - traveled;

        if (maneuver_turn_settling || remaining <= ENCODER_TURN_SETTLE_COUNTS)
        {
          motion_mode = STANDBY;
          setting_turn_speed = 0;

          if (!maneuver_turn_settling)
          {
            maneuver_turn_settling = true;
            maneuver_turn_settle_start = millis();
          }
          else if (millis() - maneuver_turn_settle_start >= TURN_SETTLE_MS)
          {
            maneuver_turn_settling = false;
            current_maneuver_index++;
            Serial.println(F("ENC TLEFT DONE"));
          }
        }
        else
        {
          motion_mode = TURNLEFT;
          setting_car_speed = 0;
          int turn_speed = (int)(remaining * ENCODER_TURN_SPEED_KP);
          if (turn_speed < TURN_SPEED_SLOW) turn_speed = TURN_SPEED_SLOW;
          if (turn_speed > TURN_SPEED_FAST) turn_speed = TURN_SPEED_FAST;
          setting_turn_speed = turn_speed;
        }
      }
    }
    else
    {
      // --- Yaw/gyro-based left turn ---
      if (motion_mode != TURNLEFT && !maneuver_turn_settling)
      {
        motion_mode = TURNLEFT;
        setting_car_speed = 0;
        captureYawHeading();
        maneuver_turn_direction_sign = 0;
        maneuver_turn_settling = false;
        setting_turn_speed = TURN_SPEED_FAST;
        rgb.flashGoldColorLeft();
      }
      {
        float yaw_delta = getYawDeltaFromHeading();

        if (maneuver_turn_direction_sign == 0 && fabs(yaw_delta) >= TURN_DIRECTION_DETECT_DEGREES)
        {
          maneuver_turn_direction_sign = yaw_delta > 0.0f ? 1 : -1;
        }

        float turn_degrees = maneuver_turn_direction_sign == 0 ? 0.0f : yaw_delta * maneuver_turn_direction_sign;
        if (turn_degrees < 0.0f)
        {
          turn_degrees = 0.0f;
        }

        turn_degrees *= TURN_ANGLE_SCALE;

        float remaining_degrees = current.value - turn_degrees;

        if (maneuver_turn_direction_sign == 0)
        {
          setting_turn_speed = TURN_SPEED_FAST;
        }
        else if (maneuver_turn_settling || remaining_degrees <= TURN_SETTLE_TOLERANCE_DEGREES)
        {
          motion_mode = STANDBY;
          setting_turn_speed = 0;

          if (!maneuver_turn_settling)
          {
            maneuver_turn_settling = true;
            maneuver_turn_settle_start = millis();
          }
          else if (millis() - maneuver_turn_settle_start >= TURN_SETTLE_MS)
          {
            maneuver_turn_direction_sign = 0;
            maneuver_turn_settling = false;
            current_maneuver_index++;
          }
        }
        else
        {
          motion_mode = TURNLEFT;
          setting_car_speed = 0;
          setting_turn_speed = getTurnSpeedCommand(remaining_degrees);
        }
      }
    }
    break;

  case MANEUVER_TURN_RIGHT:
    if (maneuver_use_encoder_turns)
    {
      // --- Encoder-based right turn ---
      if (motion_mode != TURNRIGHT && !maneuver_turn_settling)
      {
        motion_mode = TURNRIGHT;
        setting_car_speed = 0;
        maneuver_turn_settling = false;
        maneuver_initial_encoder_count = (encoder_distance_left + encoder_distance_right) / 2;
        maneuver_target_encoder_count = (int)(current.value * ENCODER_TURN_SCALE_PER_DEGREE);
        setting_turn_speed = -TURN_SPEED_FAST;
        rgb.flashGoldColorRight();
        Serial.print(F("ENC TRIGHT START tgt:"));
        Serial.println(maneuver_target_encoder_count);
      }
      {
        int current_enc = (encoder_distance_left + encoder_distance_right) / 2;
        int traveled = current_enc - maneuver_initial_encoder_count;
        int remaining = maneuver_target_encoder_count - traveled;

        if (maneuver_turn_settling || remaining <= ENCODER_TURN_SETTLE_COUNTS)
        {
          motion_mode = STANDBY;
          setting_turn_speed = 0;

          if (!maneuver_turn_settling)
          {
            maneuver_turn_settling = true;
            maneuver_turn_settle_start = millis();
          }
          else if (millis() - maneuver_turn_settle_start >= TURN_SETTLE_MS)
          {
            maneuver_turn_settling = false;
            current_maneuver_index++;
            Serial.println(F("ENC TRIGHT DONE"));
          }
        }
        else
        {
          motion_mode = TURNRIGHT;
          setting_car_speed = 0;
          int turn_speed = (int)(remaining * ENCODER_TURN_SPEED_KP);
          if (turn_speed < TURN_SPEED_SLOW) turn_speed = TURN_SPEED_SLOW;
          if (turn_speed > TURN_SPEED_FAST) turn_speed = TURN_SPEED_FAST;
          setting_turn_speed = -turn_speed;
        }
      }
    }
    else
    {
      // --- Yaw/gyro-based right turn ---
      if (motion_mode != TURNRIGHT && !maneuver_turn_settling)
      {
        motion_mode = TURNRIGHT;
        setting_car_speed = 0;
        captureYawHeading();
        maneuver_turn_direction_sign = 0;
        maneuver_turn_settling = false;
        setting_turn_speed = -TURN_SPEED_FAST;
        rgb.flashGoldColorRight();
      }
      {
        float yaw_delta = getYawDeltaFromHeading();

        if (maneuver_turn_direction_sign == 0 && fabs(yaw_delta) >= TURN_DIRECTION_DETECT_DEGREES)
        {
          maneuver_turn_direction_sign = yaw_delta > 0.0f ? 1 : -1;
        }

        float turn_degrees = maneuver_turn_direction_sign == 0 ? 0.0f : yaw_delta * maneuver_turn_direction_sign;
        if (turn_degrees < 0.0f)
        {
          turn_degrees = 0.0f;
        }

        turn_degrees *= TURN_ANGLE_SCALE;

        float remaining_degrees = current.value - turn_degrees;

        if (maneuver_turn_direction_sign == 0)
        {
          setting_turn_speed = -TURN_SPEED_FAST;
        }
        else if (maneuver_turn_settling || remaining_degrees <= TURN_SETTLE_TOLERANCE_DEGREES)
        {
          motion_mode = STANDBY;
          setting_turn_speed = 0;

          if (!maneuver_turn_settling)
          {
            maneuver_turn_settling = true;
            maneuver_turn_settle_start = millis();
          }
          else if (millis() - maneuver_turn_settle_start >= TURN_SETTLE_MS)
          {
            maneuver_turn_direction_sign = 0;
            maneuver_turn_settling = false;
            current_maneuver_index++;
          }
        }
        else
        {
          motion_mode = TURNRIGHT;
          setting_car_speed = 0;
          setting_turn_speed = -getTurnSpeedCommand(remaining_degrees);
        }
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
