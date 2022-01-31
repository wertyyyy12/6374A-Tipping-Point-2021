/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// arm_motors           motor_group   3, 8            
// left_motor           motor         1               
// right_motor          motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
float dampening = 0.65;
double wheelRadius;
double armMaxInDegrees = 150;
float goalLength; //IN INCHES

//other constants
double pi = 2 * acos(0.0);
char up[3]= "up";
char down[5]= "down";

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  left_motor.resetPosition();
  right_motor.resetPosition();
  arm_motors.setStopping(hold);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/* void driveTrainMotors: move one or both of drive train motors a certain amount

  %param leftPosition% target position of left motor, in either degrees or length units
  %param rightPosition% target position of right motor, in either degrees or length units
  %param reset% (optional; default = false) BOOL: if true, resets both drivetrain motors to original position 
  %param unitsAreDegrees% (optional; default = false) BOOL: true if units are degrees, false if units are length
    NOTE: the length units must match the units for wheelRadius

*/
void driveTrainMotors(double leftPosition, double rightPosition, bool reset = false, bool unitsAreDegrees = false) { //positions must be measured with same units as radius measure!!!
  double leftPositioninDegrees = (leftPosition * 180) / (pi * wheelRadius);  //by default, positions are not in degrees and thus have to be converted
  double rightPositioninDegrees = (rightPosition * 180) / (pi * wheelRadius); 
  if (unitsAreDegrees) {
    leftPositioninDegrees = leftPosition;
    rightPositioninDegrees = rightPosition;
  }

  left_motor.spinToPosition(leftPositioninDegrees, degrees, false);
  right_motor.spinToPosition(rightPositioninDegrees, degrees, false);
  while ((left_motor.position(degrees) < (leftPositioninDegrees - 5)) && (right_motor.position(degrees) < (rightPositioninDegrees - 5))) { //while the motors are not almost there, wait
    wait(10, msec);
  }
  if (reset) { //if reset = true, wait a bit then return to original position (stored as 0)
    wait(250, msec);
    left_motor.spinToPosition(0, degrees, false);
    right_motor.spinToPosition(0, degrees, false);
  }
}

void moveArms(char action[]) {
  if (strncmp(action, "up", 2) == 0) {
    arm_motors.spinFor(forward, armMaxInDegrees, degrees);
  }
  if (strncmp(action, "down", 2) == 0) {
    arm_motors.spinFor(forward, -armMaxInDegrees, degrees);
  } 
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //////INITIALIZATION
  arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping 
  //initialize velocities
  left_motor.setVelocity(dampening * 100, percent);
  right_motor.setVelocity(dampening * 100, percent);
  //////END INITIALIZATION
  driveTrainMotors(50, -50, true); //tap the ring into the alliance mobile goal

  //move arm up and down
  moveArms(up);
  moveArms(down);

  //turn towards correct heading
  driveTrainMotors(22.5, -22.5, false, true);

  //move forwards to mobile goal
  driveTrainMotors(goalLength, goalLength);

  //lift up the goal
  moveArms(up);

  //go back to starting position
  driveTrainMotors(0, 0, false);

  //turn around to park
  driveTrainMotors(180, -180, false);
  //
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    //DRIVER CONTROL STUFF

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
