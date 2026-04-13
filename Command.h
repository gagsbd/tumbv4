#include "EnableInterrupt.h"

// Forward declaration - full definition in Maneuver.h
void startManeuverSequence();
void stopManeuverSequence();
extern boolean maneuver_sequence_active;

char key_value = '\0';
char key_flag = '\0';
unsigned long last_button_press = 0;

void keyValue()
{
  unsigned long current_time = millis();
  if (current_time - last_button_press < 1000)
  {
    return;
  }
  last_button_press = current_time;
  
  Serial.println("BUTTON");
  
  if (maneuver_sequence_active)
  {
    stopManeuverSequence();
  }
}

bool getKeyValue()
{
  return false;
}

void keyInit()
{
  pinMode(KEY_MODE, INPUT_PULLUP);
  delay(2000);
  last_button_press = millis();
  enableInterrupt(KEY_MODE, keyValue, FALLING);
}

bool getBluetoothData()
{
  // DISABLED: Bluetooth/IR control disabled
  // Only button press (keyValue interrupt) controls maneuvers
  return false;
}
