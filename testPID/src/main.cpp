/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\akad                                             */
/*    Created:      Tue Feb 08 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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
#include "vex.h"
using namespace vex;
competition Competition;

/// --- PARAMETERS --- ///
double pi = 2 * acos(0.0);

//movement parameters
float back_arm_limit = 0;
float wheelRadius = 2; //BIG wheel radius, in inches
bool drivePIDenabled, turnPIDenabled = true;
double driveSetPoint, turnSetPoint = 0;

/// --- --- ///

//mathematical helper functions
float toDegrees(float radianMeasure) {
  return ((radianMeasure * 180) / pi);
}

bool areStringsEqual(const char str1[], const char str2[]) {
  int str1length = strlen(str1);
  int str2length = strlen(str2);
  int maxlength = std::max(str1length, str2length);
  if (strncmp(str1, str2, maxlength) == 0) {
    return true;
  }
  else {
    return false;
  }
}

//drivetrain helper functions
void resetDrivetrainMotorPositions() {
  left_motor.resetPosition();
  right_motor.resetPosition();
  left_small_motor.resetPosition();
  right_small_motor.resetPosition();
}


void spinDrivetrainMotors(double pct, const char mode[]) {
  if (areStringsEqual(mode, "Hgear")) {
    if (pct > 50) {
      left_small_motor.stop(); right_small_motor.stop();
    }

    else {
      left_small_motor.spin(forward, pct * 2, percent);
      right_small_motor.spin(forward, pct * 2, percent);
    }

    left_motor.spin(forward, pct, percent);
    right_motor.spin(forward, pct, percent);
  }
}

//drive PID
int drivePID() {
  double kP = 0.25;
  double kI = 0.0;
  double kD = 0.095;

  double error, derivative;
  double prevError = 0;

  double totalError = 0;
  
  double avgPosition, driveMotorPower;

  while (drivePIDenabled) {
    avgPosition = (left_motor.position(degrees) + right_motor.position(degrees) + (2 * (left_small_motor.position(degrees) + right_small_motor.position(degrees)))) / 4;
    error = avgPosition - driveSetPoint; //P
    totalError += error; //I
    derivative = error - prevError; //D

    driveMotorPower = (kP * error) + (kD * derivative) + (kI * totalError);
    
    spinDrivetrainMotors(driveMotorPower, "Hgear"); //this uses percentage units, tutorial used voltageUnits::volt
    
    wait(20, msec);
  }

  return 1;
}

int turnPID() {
  double kP = 0.25;
  double kI = 0.0;
  double kD = 0.095;

  double error, derivative;
  double prevError = 0;

  double totalError = 0;
  double avgPosition, driveMotorPower;

  while (turnPIDenabled) {
    avgPosition = (left_small_motor.position(degrees) + right_small_motor.position(degrees)) / 2;
    error = avgPosition - driveSetPoint; //P
    totalError += error; //I
    derivative = error - prevError; //D

    driveMotorPower = (kP * error) + (kD * derivative) + (kI * totalError);
    
    left_small_motor.spin(forward, driveMotorPower, percent);
    right_small_motor.spin(forward, -driveMotorPower, percent);
  
    wait(20, msec);
  } 

  return 1;
}

void Move(double linearDisplacement) {
  resetDrivetrainMotorPositions(); 
  double targetPositionInDegrees = toDegrees(linearDisplacement / wheelRadius);
  driveSetPoint = targetPositionInDegrees;
  task::sleep(5000); //don't allow any other driveSetPoint moving for 5 seconds 
}

void Turn(double angularDisplacement) {
  resetDrivetrainMotorPositions();
  double targetPositionInDegrees = angularDisplacement; //left -> CCW (consistent w/ math convention)
  turnSetPoint = targetPositionInDegrees;
  task::sleep(1000); //wait for the turn to complete
}


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  left_motor.resetPosition();
  right_motor.resetPosition();
  front_arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping 
  back_arm_motors.setStopping(hold);
  

}

void autonomous(void) {
  task DRIVEPID(drivePID);
  Move(2 * 12);
  wait(2, sec);
  Move(-2 * 12);
}

void usercontrol(void) {
  drivePIDenabled = false;
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  
}
