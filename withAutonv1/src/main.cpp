#include "vex.h"
#include <string.h>
#include <cmath>
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here

//decimal values that slow down the speeds of various robot parts
float drivetrain_dampening = 0.8; //for drivetrain
float front_arm_dampening = 0.35; //for front arm (two motors)
float back_arm_dampening = 0.60; //for back arm (two motors)
float dampening_interval = 0.05; //adjustment for one button press of any dampening interval

//movement parameters
float back_arm_limit = -1032.2; //in degrees -957.2   
float front_arm_limit = 1145; //in degrees
float slowdownThreshold = 75; //in degrees
float wheelRadius = 2; //in inches


//coordinate system init
float currentHeading = 90; //in degrees
float currentPosition[2] = {0, 0}; //index [0, 1] = [x, y]

//other constants
float pi = 2 * acos(0.0);


//update drive code

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

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

float maximumMagnitude(float x, float maxMag) { //cap the magnitude of "x" (in english, absolute val(x) ) at "maxMag"
  if (std::abs(x) > maxMag) { //if magnitude of x is greater than maximum magnitude...
    if (x > 0) {
      return maxMag; //cap x
    }
    else {
      return -maxMag; //cap x
    }
  }
  return x; //leave x alone 
}

float toDegrees(float radianMeasure) {
  return ((radianMeasure * 180) / pi);
}

float toRadians(float degreeMeasure) {
  return ((degreeMeasure * pi) / 180);
}

float length(float point0[2], float point1[2]) {
  float Dx = point1[0] - point0[0];
  float Dy = point1[1] - point0[1];

  return sqrt(pow(Dx, 2) + pow(Dy, 2));
}
/* void displayInfoBrain: displays a piece of information on the brain screen.

  NOTE: must be run inside of a while true loop

  %param info% the actual value of the information (ex: left_motor.position(degrees))
  %param Row% row number of starting position of display area (ex: 1)
  %param Col% column number of starting position of display area (ex: 1)
  %param Color% color of "name" portion of information  
  %param name% the display name of the information (ex: Left Motor Position)

*/
void displayInfoBrain(float info, int Row, int Col, color Color, const char* name = "") {
  // Brain.Screen.setCursor(Row, Col); 
  Brain.Screen.newLine();
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

float averageMotorPosition() {
  // return ((left_motor.position(degrees) + right_motor.position(degrees) + (2 * (left_small_motor.position(degrees) + right_small_motor.position(degrees)))) / 4);
  return ((left_motor.position(degrees) + right_motor.position(degrees)) / 2);
}

void resetDrivetrainMotorPositions() {
  left_motor.resetPosition();
  right_motor.resetPosition();
  left_small_motor.resetPosition();
  right_small_motor.resetPosition();
}

void spinDrivetrainMotors(double pct) {
  left_motor.spin(forward, pct, percent);
  right_motor.spin(forward, pct, percent);
  left_small_motor.spin(forward, maximumMagnitude(pct*2, 85), percent);
  right_small_motor.spin(forward, maximumMagnitude(pct*2, 85), percent);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  resetDrivetrainMotorPositions();
  front_arm_motors.resetPosition(); back_arm_motors.resetPosition();
  front_arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping 
  back_arm_motors.setStopping(hold);

  left_motor.setVelocity(drivetrain_dampening * 100, percent);
  right_motor.setVelocity(drivetrain_dampening * 100, percent);
  left_small_motor.setVelocity(drivetrain_dampening * 100, percent);
  right_small_motor.setVelocity(drivetrain_dampening * 100, percent);
  front_arm_motors.setVelocity(front_arm_dampening * 100, percent);
  back_arm_motors.setVelocity(back_arm_dampening * 100, percent);
}

/* void driveTrainMotors: move one or both of drive train motors a certain amount

  %param leftPosition% target position of left motor, in either degrees or length units
  %param rightPosition% target position of right motor, in either degrees or length units
  %param reset% (optional; default = false) BOOL: if true, resets both drivetrain motors to original position 
  %param unitsAreDegrees% (optional; default = false) BOOL: true if units are degrees, false if units are length
    NOTE: the length units must match the units for wheelRadius

*/

void Move(float Distance, bool preciseMovement = false) { //Distance in INCHES
  // driveTrainMotors(Distance, Distance);
  resetDrivetrainMotorPositions();
  float targetPositionInDegrees = toDegrees(Distance / wheelRadius) - (0 * Distance / std::abs(Distance));

  //u(x) = pP * iI (* dD)
  //sigmoid function to turn unbounded output into %?
  float Kp = 0.25;   //0.15 works well
  float Ki = 0.0; //0.008
  float Kd = 0.095; //0.095
  float I = 0;
  float lastError = 0; //for De calculations
  
  //asymmetrical turns to account for weird slow turning
  // if (Distance < 0) {
  //   left_motor.spinFor(forward, 15, degrees, false);
  //   right_motor.spinFor(forward, -15, degrees, false);
  // }
  // if (Distance > 0) {
  //   left_motor.spinFor(forward, -13, degrees, false);
  //   right_motor.spinFor(forward, 13, degrees, false);
  // }
  // wait(400, msec);
  while (true) {
    // Brain.Timer.reset(); //setting up the dT for I 
  //   right_motor.spinFor(forward, 23, degrees, false);
    float absoluteError = targetPositionInDegrees - averageMotorPosition();
    // float minPconst = 75;
    float P = absoluteError;

    // if (std::abs(absoluteError) < 180) {
    //   Ki = 0.0003;
    // }
    I = I + absoluteError; //integrating error fxn for integral control

    float DeltaE = absoluteError - lastError; //find DeltaE, change in error over given 20ms interval
    lastError = absoluteError;

    float controlValue = (Kp * P)  + (Ki * I) + (DeltaE * Kd);
    // if (std::abs(controlValue) < 0.25) { //avoid slowing down w/o stopping
    //   break;
    // }
    float terminationRange = 3;
    // if (!preciseMovement) {
    //   terminationRange = 12;
    // }
    if ((std::abs(targetPositionInDegrees - averageMotorPosition()) < terminationRange) ) { //stop when im close enough
      Controller1.rumble(rumbleLong);
      displayInfoController(averageMotorPosition(), 1, 1, "avg pos");
      break;
    }

    float maxAbsControl = 100; //if max > 50, small wheels cannot keep up --> with full traction on all wheels, small wheels would start sliding 
    controlValue = maximumMagnitude(controlValue, maxAbsControl);
    // if (std::abs(controlValue) > maxAbsControl) {
    //   if (controlValue > 0) {
    //     controlValue = maxAbsControl;
    //   }
    //   else {
    //     controlValue = -maxAbsControl;
    //   }
    // }
    spinDrivetrainMotors(controlValue);
    // displayInfoController(averageMotorPosition(), 1, 1, "avg pos");
    // displayInfoController(targetPositionInDegrees - averageMotorPosition(), 2, 3, "Diff"); //+ = undershoot, - = overshoot, for t > initialavg
    // displayInfoController(controlValue, 3, 3, "C Val");
    wait(20, msec); //IF CHANGING, ALL OTHER CONSTANTS MUST BE CHANGED PROPORTIONALLY
  }
  // left_motor.spin(forward, -50, percent);
  // right_motor.spin(forward, -50, percent);
  // wait(0.2, sec);
  left_motor.stop(); right_motor.stop(); left_small_motor.stop(); right_small_motor.stop();
  displayInfoController(Brain.Timer.value(), 2, 3, "STOPPED");
  displayInfoBrain(targetPositionInDegrees, 2, 3, blue, "target position");
  Controller1.rumble(rumbleShort);


  //update coords
  float Dx = Distance * cos(toRadians(currentHeading)); //good ol' polar formulae
  float Dy = Distance * sin(toRadians(currentHeading));
  currentPosition[0] = currentPosition[0] + Dx;
  currentPosition[1] = currentPosition[1] + Dy;
} 

void Turn(float angularDisplacement, bool lifting = false) {  
  // driveTrainMotors(-angularDisplacement, angularDisplacement, true);
  float turnMultiplier = 1;
  if (angularDisplacement > 0) {
    turnMultiplier = -1;
  }

  float inputToAngularDisplacementRatio = 3.25;
  if (lifting) {
    inputToAngularDisplacementRatio = 4.75;
  }

  float spinInput = inputToAngularDisplacementRatio * angularDisplacement * turnMultiplier;
  left_motor.spinFor(forward, -spinInput, degrees, false);
  left_small_motor.spinFor(forward, -spinInput * 2, degrees, false);
  right_motor.spinFor(forward, spinInput, degrees, false);
  right_small_motor.spinFor(forward, spinInput * 2, degrees, false);
  currentHeading = currentHeading + angularDisplacement; //update robot heading
  wait(1000, msec);
}

void TurnToHeading(float targetHeading) { //targetHeading in DEGREES
  float Dheading = targetHeading - currentHeading;
  if (std::abs(Dheading) > 180) {
    Dheading = 360 - Dheading;
  }
  Turn(Dheading);
}

void TurnToPoint(float targetPosition[2]) {
  float referencePosition[2] = {currentPosition[0] + 12, currentPosition[1]};

  float d = length(currentPosition, referencePosition);
  float e = length(targetPosition, referencePosition);
  float f = length(currentPosition, targetPosition);

  float targetHeading = toDegrees(acos((pow(d, 2) - pow(e, 2) + pow(f, 2)) / (2 * d * f)));
  if (currentPosition[1] > targetPosition[1]) {
    targetHeading = 360 - targetHeading;
  }
  TurnToHeading(targetHeading);
}

void MoveArms(const char arm[], const char Direction[], const char amount[] = "partial") { //maybe add 
  float reverseMultiplier = 1;
  if (areStringsEqual(Direction, "down")) {
    reverseMultiplier = -1;
  }

  float degreesLift = 150; //defaults to a partial lift
  if (areStringsEqual(amount, "full")) {
    degreesLift = 450;
  }

  if (areStringsEqual(arm, "front")) {
    front_arm_motors.spinFor(forward, degreesLift * reverseMultiplier, degrees);
  }
  if (areStringsEqual(arm, "back")) {
    back_arm_motors.spinFor(forward, degreesLift * reverseMultiplier, degrees);
  }

}

// void MMoveArms(const char arm[], const char Direction[], const char amount[]) {

// }
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
  resetDrivetrainMotorPositions();
  front_arm_motors.resetPosition(); back_arm_motors.resetPosition();
  /////-----SKILLS CHALLENGE AUTONOMOUS: STARTING SIDEWAYS----/////
  // //  / ALLIANCE: G1
  // back_arm_motors.spinToPosition(-905, degrees);
  // Move(-17.7); //lunge for 1st alliance goal
  // back_arm_motors.spinFor(forward, 700, degrees); //get alliance g1 
  // Move(10); //move forward to prepare journey to other side
  // Turn(90, false); //turn to correct heading for journey
  // Move(75); //129 predicted, 117.45 to avoid overshooting
  // Move(-10);
  //     //Turn around, get rid of G1 and possess G2
  // Turn(80, false);
  // Move(40);

  /////-----SKILLS CHALLENGE END-----/////

  ////----AUTONOMOUS-----/////
  //deploy arms
  front_arm_motors.spinFor(forward, 60, degrees);
  front_arm_motors.spinFor(forward, -60, degrees);
  //OPTION A: LEFT GOAL
  //turn towards nearest neutral goal
  Turn(7.5);
  //move and grab the nearest neutral goal
  Move(50);
  MoveArms("front", "up");
  //go back from the nearest neutral
  Move(-40);

  // //OPTION B: CENTER GOAL
  // Turn(20);
  

  /////-----/////




  
  
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
  //display port info ONCE
  // displayInfoBrain(left_motor.position(degrees), 2, 3, cyan, "(Port 1B, 7F) Left Drivetrain Motor");
  // displayInfoBrain(right_motor.position(degrees), 3, 3, cyan, "(Port 2B, 8F) Right Drivetrain Motor");
  // displayInfoBrain(front_arm_motors.position(degrees), 4, 3, cyan, "(Ports 3L, 4R) Front Arm Motors");
  // displayInfoBrain(back_arm_motors.position(degrees), 5, 3, cyan, "(Ports 5L, 6R) Back Arm Motors");
  
  //enables multiplier adjustment
  //   //up, down buttons -> increase, decrease drivetrain dampening multiplier   
  // Controller1.ButtonUp.pressed([]() {drivetrain_dampening = drivetrain_dampening + dampening_interval; displayInfoController(drivetrain_dampening, 1, 1, "Drivetrain");});
  // Controller1.ButtonDown.pressed([]() {drivetrain_dampening = drivetrain_dampening - dampening_interval; displayInfoController(drivetrain_dampening, 1, 1, "Drivetrain");});
  //   //left, right buttons -> decrease, increase front arm dampening multiplier
  // Controller1.ButtonLeft.pressed([]() {front_arm_dampening = front_arm_dampening - dampening_interval; displayInfoController(front_arm_dampening, 2, 1, "Front Arm");});
  // Controller1.ButtonRight.pressed([]() {front_arm_dampening = front_arm_dampening + dampening_interval; displayInfoController(front_arm_dampening, 2, 1, "Front Arm");});
  //   //B, X buttons -> decrease, increase back arm dampening multiplier
  // Controller1.ButtonB.pressed([]() {back_arm_dampening = back_arm_dampening - dampening_interval; displayInfoController(back_arm_dampening, 3, 1, "Back Arm");});
  // Controller1.ButtonX.pressed([]() {back_arm_dampening = back_arm_dampening + dampening_interval; displayInfoController(back_arm_dampening, 3, 1, "Back Arm");});
  
  Controller1.ButtonY.pressed([]() {drivetrain_dampening = 0; back_arm_dampening = 0; front_arm_dampening = 0;});
  while (1) {

    float axis3Val = Controller1.Axis3.position() * drivetrain_dampening;
    float axis2Val = Controller1.Axis2.position() * drivetrain_dampening;
  
    
    left_motor.spin(forward, axis3Val, percent); //spin drivetrain
    right_motor.spin(forward, axis2Val, percent);

    // double leftMotorVel = axis3Val * 2;
    // double rightMotorVel = axis2Val * 2;

    left_small_motor.spin(forward, maximumMagnitude(axis3Val * 2, 85), percent);
    right_small_motor.spin(forward, maximumMagnitude(axis2Val * 2, 85), percent);

    float back_forward = back_arm_dampening * 100 * Controller1.ButtonR1.pressing(); //R1 -> back arm forward
    float back_reverse = back_arm_dampening * 100 * Controller1.ButtonR2.pressing(); //R2 -> back arm reverse
    float backVel = back_forward - back_reverse;
    if (back_arm_motors.position(degrees) + (slowdownThreshold / (backVel / 100)) < back_arm_limit) { //if the front arms are going to spin into the limit, then disable them
      backVel = 0;
    }
    back_arm_motors.spin(forward, backVel, percent); //spin back arm

    float front_forward = front_arm_dampening * 100 * Controller1.ButtonL1.pressing(); //L1 -> front arm forward
    float front_reverse = front_arm_dampening * 100 * Controller1.ButtonL2.pressing(); //L2 -> front arm reverse
    float frontVel = front_forward - front_reverse;
    std::cout << front_arm_motors.position(degrees) << std::endl;
    if (front_arm_motors.position(degrees) + (slowdownThreshold / (frontVel / 100)) > front_arm_limit) { //if the front arms are going to spin into the limit, then disable them
      frontVel = 0;
    }
    front_arm_motors.spin(forward, frontVel, percent); //spin front arm   912 degrees - 15 degrees 
    

    // displayInfoBrain(back_arm_motors.position(degrees), 6, 3, cyan, "BACK ARM POS");
    // displayInfoController((axis2Val * 2) - (axis3Val * 2), 1, 1, "axis diff");
    // displayInfoController(left_small_motor.velocity(percent), 1, 1, "LSM spd");
    // displayInfoController(left_motor.velocity(percent), 2, 1, "LM spd");

    // displayInfoController(back_arm_motors.position(degrees), 1, 1, "Barm pos");
    // displayInfoController(front_arm_motors.position(degrees), 2, 1, "Farm pos");

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
