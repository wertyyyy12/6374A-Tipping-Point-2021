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
float armMax = 0.35;
int goalLengthInDegrees = 1630;
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
void driveTrainMotors(int leftPosition, int rightPosition, bool reset) {
  left_motor.spinToPosition(leftPosition, degrees, false);
  right_motor.spinToPosition(rightPosition, degrees, false);
  while ((left_motor.position(degrees) < (leftPosition - 5)) && (right_motor.position(degrees) < (leftPosition - 5))) {
    wait(10, msec);
  }
  if (reset) {
    wait(250, msec);
    left_motor.spinToPosition(0, degrees, false);
    right_motor.spinToPosition(0, degrees, false);
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
  arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping ;
  //initialize velocities
  left_motor.setVelocity(dampening * 100, percent);
  right_motor.setVelocity(dampening * 100, percent);
  int goalLengthInDegrees = 1630;
  //////END INITIALIZATION
  driveTrainMotors(50, -50, true); //tap the ring into the alliance mobile goal

  //move arm up and down
  arm_motors.spinFor(forward, 150, degrees);
  arm_motors.spinFor(forward, -150, degrees);

  //turn towards correct heading
  driveTrainMotors(22.5, -22.5, false);

  //move forwards to mobile goal
  driveTrainMotors(goalLengthInDegrees, goalLengthInDegrees, false);

  //lift up the goal
  arm_motors.spinFor(forward, 150, degrees);

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
    left_motor.spin(forward, Controller1.Axis3.position() * dampening, percent);
    right_motor.spin(forward, Controller1.Axis2.position() * dampening, percent);
    //NOTE: if any direction needs to be reversed, do so in the robot configuration (reverse the motor)

    //if L1 is pressed, spin the arm motors forward
    if (Controller1.ButtonL1.pressing()) {
      arm_motors.spin(forward, armMax * 100, percent);
    }

    //if L2 is pressed, spin the arm motors the other way
    else if (Controller1.ButtonL2.pressing()) {
      arm_motors.spin(reverse, armMax * 100, percent);
    }

    else {
      arm_motors.stop();
    }

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
