// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// front_arm_motors     motor_group   3, 4            
// left_motor           motor         1               
// right_motor          motor         2               
// back_arm_motors      motor_group   5, 6            
// left_small_motor     motor         7               
// right_small_motor    motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// front_arm_motors     motor_group   3, 4            
// left_motor           motor         1               
// right_motor          motor         2               
// back_arm_motors      motor_group   5, 6            
// left_small_motor     motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Akilan Babu                                               */
/*    Created:      Tue Nov 16 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//CHANGE THE PORT DISPLAY ON BRAIN AFTER CHANGING ANY PORTS
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// front_arm_motors     motor_group   3, 4            
// left_motor           motor         1               
// right_motor          motor         2               
// back_arm_motors      motor_group   5, 6            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

#include <iostream>
#include <string>
#include <cmath>

using namespace vex; 

/*NOTE on comment formatting: button controls are explained with "arrow syntax"
Examples:
  button A -> robot spins 180 degrees                      when button A is pressed, the robot spins 180 degrees
  button A, B -> robot spins 180 degrees, -180 degrees     when button A is pressed, robot spins 180 degrees. when button B is pressed, robot spins -180 degrees
*/

//decimal values that slow down the speeds of various robot parts
float drivetrain_dampening = 0.65; //for drivetrain
float front_arm_dampening = 0.35; //for front arm (two motors)
float back_arm_dampening = 0.60; //for back arm (two motors)
float dampening_interval = 0.05; //adjustment for one button press of any dampening interval

float back_arm_limit = 100;
float front_arm_limit = 235;
/* void displayInfoBrain: displays a piece of information on the brain screen.

  NOTE: must be run inside of a while true loop

  %param info% the actual value of the information (ex: left_motor.position(degrees))
  %param Row% row number of starting position of display area (ex: 1)
  %param Col% column number of starting position of display area (ex: 1)
  %param Color% color of "name" portion of information  
  %param name% the display name of the information (ex: Left Motor Position)

*/
void displayInfoBrain(float info, int Row, int Col, color Color, const char* name = "") {
  Brain.Screen.setCursor(Row, Col); 
  Brain.Screen.setPenColor(Color);
  Brain.Screen.print(name);
  Brain.Screen.print(" ");
  Brain.Screen.setPenColor(white);
  Brain.Screen.print(info);
}

//almost the same function as above, for the controller
//no color because controller is one color LCD display
void displayInfoController(float info, int Row, int Col, const char* name = "") {
  Controller1.Screen.setCursor(Row, Col); 
  Controller1.Screen.print(name);
  Controller1.Screen.print(" ");
  Controller1.Screen.print(info);

}

void resetDrivetrainMotorPositions() {
  left_motor.resetPosition();
  right_motor.resetPosition();
  left_small_motor.resetPosition();
  right_small_motor.resetPosition();
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit(); resetDrivetrainMotorPositions();
// User control code here, inside the loop
  //display port info ONCE
  displayInfoBrain(left_motor.position(degrees), 2, 3, cyan, "(Ports 1B, 7F) Left Drivetrain Motor");
  displayInfoBrain(right_motor.position(degrees), 3, 3, cyan, "(Ports 2B, 8F) Right Drivetrain Motor");
  displayInfoBrain(front_arm_motors.position(degrees), 4, 3, cyan, "(Ports 3L, 4R) Front Arm Motors");
  displayInfoBrain(back_arm_motors.position(degrees), 5, 3, cyan, "(Ports 5L, 6R) Back Arm Motors");
  
  //enables multiplier adjustment
  //   //up, down buttons -> increase, decrease drivetrain dampening multiplier   
  Controller1.ButtonUp.pressed([]() {drivetrain_dampening = drivetrain_dampening + dampening_interval; displayInfoController(drivetrain_dampening, 1, 1, "Drivetrain");});
  Controller1.ButtonDown.pressed([]() {drivetrain_dampening = drivetrain_dampening - dampening_interval; displayInfoController(drivetrain_dampening, 1, 1, "Drivetrain");});
    //left, right buttons -> decrease, increase front arm dampening multiplier
  Controller1.ButtonLeft.pressed([]() {front_arm_dampening = front_arm_dampening - dampening_interval; displayInfoController(front_arm_dampening, 2, 1, "Front Arm");});
  Controller1.ButtonRight.pressed([]() {front_arm_dampening = front_arm_dampening + dampening_interval; displayInfoController(front_arm_dampening, 2, 1, "Front Arm");});
    //B, X buttons -> decrease, increase back arm dampening multiplier
  Controller1.ButtonB.pressed([]() {back_arm_dampening = back_arm_dampening - dampening_interval; displayInfoController(back_arm_dampening, 3, 1, "Back Arm");});
  Controller1.ButtonX.pressed([]() {back_arm_dampening = back_arm_dampening + dampening_interval; displayInfoController(back_arm_dampening, 3, 1, "Back Arm");});
  while (1) {

    float axis3Val = Controller1.Axis3.position() * drivetrain_dampening;
    float axis2Val = Controller1.Axis2.position() * drivetrain_dampening;

    
    left_motor.spin(forward, axis3Val, percent); //spin drivetrain
    right_motor.spin(forward, axis2Val, percent);
    left_small_motor.spin(forward, axis3Val * 2, percent); //it will slide at >= 50% but oh the hell well
    right_small_motor.spin(forward, axis2Val * 2, percent);


    int back_forward = back_arm_dampening * 100 * Controller1.ButtonR1.pressing(); //R1 -> back arm forward
    int back_reverse = back_arm_dampening * 100 * Controller1.ButtonR2.pressing(); //R2 -> back arm reverse

    int front_forward = front_arm_dampening * 100 * Controller1.ButtonL1.pressing(); //L1 -> front arm forward
    int front_reverse = front_arm_dampening * 100 * Controller1.ButtonL2.pressing(); //L2 -> front arm reverse


    //if within limits, spin the arms
    if (std::abs(back_arm_motors.position(degrees)) <= back_arm_limit) {
      back_arm_motors.spin(forward,  back_forward - back_reverse, percent); //spin back arm
    }

    if (std::abs(front_arm_motors.position(degrees)) <= front_arm_limit) {
      front_arm_motors.spin(forward,  front_forward - front_reverse, percent); //spin front arm
    }

    

    // displayInfoBrain(back_arm_motors.position(degrees), 6, 3, cyan, "BACK ARM POS");
    // displayInfoController(front_arm_motors.torque(InLb), 2, 3, "F arm torque");

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}
