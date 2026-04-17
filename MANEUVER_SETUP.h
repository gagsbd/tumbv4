/*
 * HOW TO USE THE SIMPLIFIED MANEUVER SYSTEM
 * ==========================================
 * 
 * 1. Define your maneuvers in the initManeuvers() function (below)
 * 2. Call initManeuvers() from setup()
 * 3. Press '*' button to start execution
 * 4. Press '#' button to stop
 * 
 * That's it! Super simple.
 */

// ===== DEFINE YOUR MANEUVERS HERE =====
void initManeuvers()
{
  // Define your maneuver sequence here
  // This runs once when the robot starts
  
  clearManeuvers();
  
  // Add your maneuvers:
  // For FORWARD/BACKWARD: value = distance in mm
  // For TURN: value = angle in degrees
  // addManeuver(MANEUVER_FORWARD, 500);       // Move forward 500 mm
  // addManeuver(MANEUVER_TURN_RIGHT, 90);      // Turn left 90 degrees using MPU yaw
   //addManeuver(MANEUVER_TURN_RIGHT, 90); 
   //addManeuver(MANEUVER_TURN_RIGHT, 90); 
   //addManeuver(MANEUVER_TURN_RIGHT, 90); 
 // addManeuver(MANEUVER_FORWARD, 500);       // Move forward 500 encoder counts
 // addManeuver(MANEUVER_TURN_LEFT, 90);    // Turn right for 500ms
 // addManeuver(MANEUVER_FORWARD, 500); 
   addManeuver(MANEUVER_TURN_LEFT, 90); 
  // addManeuver(MANEUVER_BACKWARD, 500);
  
  Serial.print(F("Maneuvers: "));
  Serial.println(maneuver_count);
}
// ======================================

/*
 * INSTRUCTIONS:
 * =============
 * 
 * 1. Edit the initManeuvers() function above
 * 
 * 2. In setup(), add this line:
 *    initManeuvers();
 * 
 * 3. Save and upload
 * 
 * 4. Press * button to start maneuvers
 * 
 * 5. Press # button to stop
 * 
 * Example maneuver definitions:
 * =============================
 * 
 * Forward and Backward:
 *   clearManeuvers();
 *   addManeuver(MANEUVER_FORWARD, 500);
 *   addManeuver(MANEUVER_BACKWARD, 500);
 * 
 * Square Pattern:
 *   clearManeuvers();
 *   addManeuver(MANEUVER_FORWARD, 500);
 *   addManeuver(MANEUVER_TURN_LEFT, 90);
 *   addManeuver(MANEUVER_FORWARD, 500);
 *   addManeuver(MANEUVER_TURN_LEFT, 90);
 *   addManeuver(MANEUVER_FORWARD, 500);
 *   addManeuver(MANEUVER_TURN_LEFT, 90);
 *   addManeuver(MANEUVER_FORWARD, 500);
 *   addManeuver(MANEUVER_TURN_LEFT, 90);
 * 
 * Simple Move:
 *   clearManeuvers();
 *   addManeuver(MANEUVER_FORWARD, 500);
 */
