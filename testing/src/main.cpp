/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\akad                                             */
/*    Created:      Tue Nov 16 2021                                           */
/*    Description:  V5 project                                                */
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

//value between 0 and 1, directly proportional to maximum motor power
float dampening = 0.65;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  while (true) {
    left_motor.spin(forward, Controller1.Axis3.position() * dampening, percent);
    right_motor.spin(forward, Controller1.Axis2.position() * dampening, percent);
    //NOTE: if any direction needs to be reversed, do so in the robot configuration (reverse the motor)

    wait(20, msec);
  }

}
