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

//this is a (decimal) value b/w 0 and 1. lower the value to lower max motor power (and make drive train slower)
float dampening = 0.65;
//0.05

//this is a percentage value b/w 0 and 100, determines percent maximum of arm power
float armMax = 0.35;
//0.01


//2 in


//NOTE: you may have to switch the ports of the motors in robot configuration
//void increaseDriveTrainMultiplier() {
//  dampening = dampening + dampeningInterval;
//}

//void decreaseDriveTrainMultiplier() {
  //dampening = dampening - dampeningInterval;
//}

//void increaseArmMotorMultiplier() {
  //armMax = armMax + armInterval;
//}

//void decreaseArmMotorMultiplier() {
  //armMax = armMax - armInterval;
//}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping 
    //button up/button down = increase/decrease drivetrain multiplier
  //Controller1.ButtonUp.pressed(increaseDriveTrainMultiplier);
  //Controller1.ButtonDown.pressed(decreaseDriveTrainMultiplier);

  //button A/B = increase/decrease arm multiplier 
  //Controller1.ButtonA.pressed(increaseArmMotorMultiplier);
  //Controller1.ButtonB.pressed(decreaseArmMotorMultiplier);
  while (true) {
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



    wait(20,msec);
  }

}
