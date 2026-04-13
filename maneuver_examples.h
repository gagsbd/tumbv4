/*
 * EXAMPLE MANEUVER SEQUENCES
 * ================================
 * 
 * This file contains ready-to-use example maneuver sequences.
 * You can copy these into your code to test different movement patterns.
 * 
 * HOW TO USE THESE EXAMPLES:
 * 1. Call the appropriate function from your setup() or when needed
 * 2. Make sure to call startManeuverSequence() after setting up maneuvers
 * 3. Adjust the parameters (encoder counts, durations) based on your robot's calibration
 */

// EXAMPLE 1: Simple Forward Movement
void maneuver_example_forward()
{
  clearManeuvers();
  addManeuver(MANEUVER_FORWARD, 500);  // Move forward
  startManeuverSequence();
}

// EXAMPLE 2: Simple Backward Movement
void maneuver_example_backward()
{
  clearManeuvers();
  addManeuver(MANEUVER_BACKWARD, 500);  // Move backward
  startManeuverSequence();
}

// EXAMPLE 3: Turn Left
void maneuver_example_turn_left()
{
  clearManeuvers();
  addManeuver(MANEUVER_TURN_LEFT, 500);  // Turn left for 500ms
  startManeuverSequence();
}

// EXAMPLE 4: Turn Right
void maneuver_example_turn_right()
{
  clearManeuvers();
  addManeuver(MANEUVER_TURN_RIGHT, 500);  // Turn right for 500ms
  startManeuverSequence();
}

// EXAMPLE 5: Forward Then Backward (back and forth)
void maneuver_example_back_and_forth()
{
  clearManeuvers();
  addManeuver(MANEUVER_FORWARD, 500);   // Go forward
  addManeuver(MANEUVER_WAIT, 500);      // Wait half second
  addManeuver(MANEUVER_BACKWARD, 500);  // Go backward
  addManeuver(MANEUVER_WAIT, 500);      // Wait half second
  startManeuverSequence();
}

// EXAMPLE 6: Square Pattern (180cm per side)
void maneuver_example_square()
{
  clearManeuvers();
  // First side (forward)
  addManeuver(MANEUVER_FORWARD, 500);
  addManeuver(MANEUVER_TURN_LEFT, 500);
  // Second side (forward)
  addManeuver(MANEUVER_FORWARD, 500);
  addManeuver(MANEUVER_TURN_LEFT, 500);
  // Third side (forward)
  addManeuver(MANEUVER_FORWARD, 500);
  addManeuver(MANEUVER_TURN_LEFT, 500);
  // Fourth side (forward)
  addManeuver(MANEUVER_FORWARD, 500);
  addManeuver(MANEUVER_TURN_LEFT, 500);
  startManeuverSequence();
}

// EXAMPLE 7: Zigzag Pattern
void maneuver_example_zigzag()
{
  clearManeuvers();
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 300);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_RIGHT, 300);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 300);
  addManeuver(MANEUVER_FORWARD, 300);
  startManeuverSequence();
}

// EXAMPLE 8: Triangle Pattern
void maneuver_example_triangle()
{
  clearManeuvers();
  // Side 1
  addManeuver(MANEUVER_FORWARD, 400);
  addManeuver(MANEUVER_TURN_LEFT, 600);  // 120-degree turn (approx)
  // Side 2
  addManeuver(MANEUVER_FORWARD, 400);
  addManeuver(MANEUVER_TURN_LEFT, 600);  // 120-degree turn
  // Side 3
  addManeuver(MANEUVER_FORWARD, 400);
  addManeuver(MANEUVER_TURN_LEFT, 600);  // 120-degree turn (back to start)
  startManeuverSequence();
}

// EXAMPLE 9: Spin in Place
void maneuver_example_spin()
{
  clearManeuvers();
  addManeuver(MANEUVER_TURN_RIGHT, 1500);  // Full rotation (1500ms)
  startManeuverSequence();
}

// EXAMPLE 10: Complex Movement - Forward, Turn, Backward, Turn
void maneuver_example_complex()
{
  clearManeuvers();
  addManeuver(MANEUVER_FORWARD, 400);     // Move forward
  addManeuver(MANEUVER_WAIT, 500);        // Wait
  addManeuver(MANEUVER_TURN_LEFT, 400);   // Turn left
  addManeuver(MANEUVER_WAIT, 500);        // Wait
  addManeuver(MANEUVER_BACKWARD, 300);    // Move backward
  addManeuver(MANEUVER_WAIT, 500);        // Wait
  addManeuver(MANEUVER_TURN_RIGHT, 400);  // Turn right
  addManeuver(MANEUVER_WAIT, 500);        // Wait
  addManeuver(MANEUVER_FORWARD, 400);     // Move forward again
  startManeuverSequence();
}

// EXAMPLE 11: Figure-8 Pattern
void maneuver_example_figure8()
{
  clearManeuvers();
  // Left circle
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_LEFT, 400);
  // Right circle
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_RIGHT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_RIGHT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_RIGHT, 400);
  addManeuver(MANEUVER_FORWARD, 300);
  addManeuver(MANEUVER_TURN_RIGHT, 400);
  startManeuverSequence();
}

// EXAMPLE 12: Precision Test (small movements)
void maneuver_example_precision()
{
  clearManeuvers();
  addManeuver(MANEUVER_FORWARD, 100);   // Small forward
  addManeuver(MANEUVER_WAIT, 300);      // Pause
  addManeuver(MANEUVER_FORWARD, 100);   // Another small forward
  addManeuver(MANEUVER_TURN_LEFT, 300);  // Small turn
  addManeuver(MANEUVER_FORWARD, 100);   // Small forward
  startManeuverSequence();
}

/*
 * HOW TO USE THESE EXAMPLES:
 * 
 * Option 1: Call from setup()
 *   void setup() {
 *     Serial.begin(9600);
 *     ultrasonicInit();
 *     keyInit();
 *     rgb.initialize();
 *     voltageInit();
 *     start_prev_time = millis();
 *     carInitialize();
 *     
 *     // Delay to let robot balance
 *     delay(2000);
 *     
 *     // Start maneuver sequence
 *     maneuver_example_square();
 *   }
 * 
 * Option 2: Call when a key is pressed
 *   Add this to keyEventHandle():
 *     case '1':
 *       maneuver_example_square();
 *       break;
 *     case '2':
 *       maneuver_example_zigzag();
 *       break;
 * 
 * Option 3: Call from remote control
 *   Use serial commands or Bluetooth to trigger different patterns
 * 
 * CALIBRATION TIPS:
 * 
 * 1. Test FORWARD movement:
 *    - Create a simple example with just addManeuver(MANEUVER_FORWARD, 100);
 *    - Run and measure actual distance
 *    - Adjust the value (100, 200, 300, etc.) to match desired distance
 * 
 * 2. Test TURN movement:
 *    - Create a simple example with just addManeuver(MANEUVER_TURN_LEFT, 100);
 *    - Run and measure rotation angle
 *    - Adjust the value (100, 200, 300, etc.) to match desired angle
 * 
 * 3. Common ranges (adjust based on your robot):
 *    - Forward/Backward: 100-1000 encoder counts
 *    - Turn: 200-1000 milliseconds
 *    - Wait: 300-5000 milliseconds
 */
