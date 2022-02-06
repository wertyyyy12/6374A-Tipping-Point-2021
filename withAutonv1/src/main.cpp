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
#include <string.h>
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here

//decimal values that slow down the speeds of various robot parts
float drivetrain_dampening = 0.85; //for drivetrain
float front_arm_dampening = 0.35; //for front arm (two motors)
float back_arm_dampening = 0.60; //for back arm (two motors)
float dampening_interval = 0.05; //adjustment for one button press of any dampening interval

//movement parameters
float back_arm_limit = 0;
float wheelRadius = 2; //in inches
float armMaxInDegrees = 150;

//coordinate system init
float currentHeading = 90; //in degrees
float currentPosition[2] = {0, 0}; //index [0, 1] = [x, y]

//other constants
float pi = 2 * acos(0.0);


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

float averageMotorPosition() {
  return ((left_motor.position(degrees) + right_motor.position(degrees)) / 2);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

}

/* void driveTrainMotors: move one or both of drive train motors a certain amount

  %param leftPosition% target position of left motor, in either degrees or length units
  %param rightPosition% target position of right motor, in either degrees or length units
  %param reset% (optional; default = false) BOOL: if true, resets both drivetrain motors to original position 
  %param unitsAreDegrees% (optional; default = false) BOOL: true if units are degrees, false if units are length
    NOTE: the length units must match the units for wheelRadius

*/
void driveTrainMotors(float leftPosition, float rightPosition, bool unitsAreDegrees = false) { //positions must be measured with same units as radius measure!!!
  float leftPositioninDegrees = toDegrees(leftPosition / wheelRadius);  //by default, positions are not in degrees and thus have to be converted to inches
  float rightPositioninDegrees = toDegrees(rightPosition / wheelRadius); 
  if (unitsAreDegrees) {
    leftPositioninDegrees = leftPosition;
    rightPositioninDegrees = rightPosition;
  }

  while ((std::abs(left_motor.position(degrees) - leftPositioninDegrees) > 10) || (std::abs(right_motor.position(degrees) - rightPositioninDegrees) > 10)) {
    // float leftVelocityPercent, rightVelocityPercent;
    float proportionalConstant = 1;
    float leftProportionalError = proportionalConstant * 100 * ((leftPositioninDegrees - left_motor.position(degrees)) / (leftPositioninDegrees));
    float rightProportionalError = proportionalConstant * 100 * ((rightPositioninDegrees - right_motor.position(degrees)) / (rightPositioninDegrees));
    
    left_motor.spin(forward, leftProportionalError, percent);
    right_motor.spin(forward, rightProportionalError, percent);

    // displayInfoController(left_motor.position(degrees), 2, 3, "L POS");
    // displayInfoController(right_motor.position(degrees), 3, 3, "R POS");
    displayInfoController(std::abs(left_motor.position(degrees) - leftPositioninDegrees), 2, 3, "L DIFF");
    displayInfoController(std::abs(right_motor.position(degrees) - rightPositioninDegrees), 3, 3, "R DIFF");


    wait(10, msec); 
  }
  
  left_motor.stop();
  right_motor.stop();



} //basically retired

void Move(float Distance) { //Distance in INCHES
  // driveTrainMotors(Distance, Distance);
  float targetPositionInDegrees = toDegrees(Distance / wheelRadius);

  //u(x) = pP * iI (* dD)
  //sigmoid function to turn unbounded output into %?
  float Pconstant = 0.15;   //0.15 works well
  float Iconstant = 0; //0.008
  float Dconstant = 0.0;
  float I = 0;
  float e0 = 0; //for De calculations
  

  while ((std::abs(targetPositionInDegrees - averageMotorPosition()) > 10)) {
    // Brain.Timer.reset(); //setting up the dT for I 
    float absoluteError = targetPositionInDegrees - averageMotorPosition();
    // float minPconst = 75;
    float P = Pconstant * absoluteError;

    if (std::abs(absoluteError) < 180) {
      Iconstant = 0;
    }
    I = I + (Iconstant * absoluteError); //integrating error fxn for integral control

    float DeltaE = absoluteError - e0; //find DeltaE, change in error over given 10ms interval
    e0 = absoluteError;

    float controlValue = P + I + (DeltaE * Dconstant);
    left_motor.spin(forward, controlValue, percent);
    right_motor.spin(forward, controlValue, percent);
    displayInfoController(averageMotorPosition() - targetPositionInDegrees, 1, 3, "Diff");
    displayInfoController(controlValue, 3, 3, "C Val");
    wait(20, msec); //IF CHANGING, ALL OTHER CONSTANTS MUST BE CHANGED PROPORTIONALLY
  }
  left_motor.spin(forward, -50, percent);
  right_motor.spin(forward, -50, percent);
  wait(0.2, sec);
  left_motor.stop();
  right_motor.stop();
  displayInfoController(696969, 2, 3, "STOPPED");
  Controller1.rumble(rumbleShort);


  //update coords
  faloat Dx = Distance * cos(toRadians(currentHeading)); //good ol' polar formulae
  float Dy = Distance * sin(toRadians(currentHeading));
  currentPosition[0] = currentPosition[0] + Dx;
  currentPosition[1] = currentPosition[1] + Dy;
} 

void Turn(float angularDisplacement) {  
  driveTrainMotors(-angularDisplacement, angularDisplacement, true);
  currentHeading = currentHeading + angularDisplacement; //update robot heading
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

void MoveArms(const char arm[], const char action[]) { //maybe add 
  float reverseMultiplier = 1;
  if (areStringsEqual(action, "down")) {
    reverseMultiplier = -1;
  }

  if (areStringsEqual(arm, "front")) {
    front_arm_motors.spinFor(forward, armMaxInDegrees * reverseMultiplier, degrees);
  }
  if (areStringsEqual(arm, "back")) {
    back_arm_motors.spinFor(forward, armMaxInDegrees * reverseMultiplier, degrees);
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

  left_motor.resetPosition();
  right_motor.resetPosition();
  front_arm_motors.setStopping(hold); //ensures that the arms stay up in place instead of dropping 
  back_arm_motors.setStopping(hold);

  left_motor.setVelocity(drivetrain_dampening * 100, percent);
  right_motor.setVelocity(drivetrain_dampening * 100, percent);
  front_arm_motors.setVelocity(front_arm_dampening * 100, percent);
  back_arm_motors.setVelocity(back_arm_dampening * 100, percent);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  //////END INITIALIZATION
  // Turn(-50); wait(2500, msec); Turn(50);

  // //move arm up and down
  // MoveArms("front", "up");
  // MoveArms("front", "down");

  // //turn towards correct heading
  // Turn(-1.5);

  //move forwards to mobile goal
  // Move(goalLength);

  Move(12 * 5);
  // wait(3, sec);
  // Move(12 * 3);


  // //lift up the goal
  // MoveArms("front", "up");

  // //go back to starting position
  // driveTrainMotors(0, 0, false);

  // //turn around to park
  // driveTrainMotors(180, -180, false);
  
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
  displayInfoBrain(left_motor.position(degrees), 2, 3, cyan, "(Port 1) Left Drivetrain Motor");
  displayInfoBrain(right_motor.position(degrees), 3, 3, cyan, "(Port 2) Right Drivetrain Motor");
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
    left_motor.spin(forward, Controller1.Axis3.position() * drivetrain_dampening, percent); //spin drivetrain
    right_motor.spin(forward, Controller1.Axis2.position() * drivetrain_dampening, percent);

        
    int back_forward = back_arm_dampening * 100 * Controller1.ButtonR1.pressing(); //R1 -> back arm forward
    int back_reverse = back_arm_dampening * 100 * Controller1.ButtonR2.pressing(); //R2 -> back arm reverse

    int front_forward = front_arm_dampening * 100 * Controller1.ButtonL1.pressing(); //L1 -> front arm forward
    int front_reverse = front_arm_dampening * 100 * Controller1.ButtonL2.pressing(); //L2 -> front arm reverse

    back_arm_motors.spin(forward,  back_forward - back_reverse, percent); //spin back arm
    front_arm_motors.spin(forward,  front_forward - front_reverse, percent); //spin front arm

    displayInfoBrain(back_arm_motors.position(degrees), 6, 3, cyan, "BACK ARM POS");

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
