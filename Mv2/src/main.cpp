/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Akilan Babu                                               */
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

#include <iostream>
#include <string>

using namespace vex;

//this is a (decimal) value b/w 0 and 1. lower the value to lower max motor power (and make drive train slower)
float drivetrain_dampening = 0.65;
//0.05

//this is a percentage value b/w 0 and 100, determines percent maximum of arm power
float arm_dampening = 0.35;
//0.01


//2 in


//NOTE: you may have to switch the ports of the motors in robot configuration
//void increaseDriveTrainMultiplier() {
//  drivetrain_dampening = drivetrain_dampening + dampeningInterval;
//}

//void decreaseDriveTrainMultiplier() {
  //drivetrain_dampening = drivetrain_dampening - dampeningInterval;
//}

//void increaseArmMotorMultiplier() {
  //arm_dampening = arm_dampening + armInterval;
//}

//void decreaseArmMotorMultiplier() {
  //arm_dampening = arm_dampening - armInterval;
//}


/* void displayInfo: displays a piece of information on the brain screen.

  NOTE: must be run inside of a while true loop

  %param name% the display name of the information (ex: Left Motor Position)
  %param info% the actual value of the information (ex: left_motor.position(degrees))
  %param Row% row number of starting position of display area (ex: 1)
  %param Col% column number of starting position of display area (ex: 1) 

*/
void displayInfo(const char* name, float info, int Row, int Col) {
  Brain.Screen.setCursor(Row, Col); 
  Brain.Screen.setPenColor(cyan);
  Brain.Screen.print(name);
  Brain.Screen.print(" ");
  Brain.Screen.setPenColor(white);
  Brain.Screen.print(info);
}

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
  Brain.Screen.print("Motor Positions (in degrees)");
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Other Information");
  while (true) {
    left_motor.spin(forward, Controller1.Axis3.position() * drivetrain_dampening, percent);
    right_motor.spin(forward, Controller1.Axis2.position() * drivetrain_dampening, percent);

    //display useful information
    displayInfo("Left Drivetrain Motor", left_motor.position(degrees), 2, 3);
    displayInfo("Right Drivetrain Motor", right_motor.position(degrees), 3, 3);
    displayInfo("Arm Motors", arm_motors.position(degrees), 4, 3); //doesnt work properly unless BOTH motors are connected
    displayInfo("Drivetrain Dampening Multiplier", drivetrain_dampening, 6, 3);
    displayInfo("Arm Dampening Multiplier", drivetrain_dampening, 7, 3);
    

    //NOTE: if any direction needs to be reversed, do so in the robot configuration (reverse the motor)

    //if L1 is pressed, spin the arm motors forward
    if (Controller1.ButtonL1.pressing()) {  
      arm_motors.spin(forward, arm_dampening * 100, percent);
    }

    //if L2 is pressed, spin the arm motors the other way
    else if (Controller1.ButtonL2.pressing()) {
      arm_motors.spin(reverse, arm_dampening * 100, percent);
    }

    else {
      arm_motors.stop();
    }



    wait(20,msec);
  }

}
