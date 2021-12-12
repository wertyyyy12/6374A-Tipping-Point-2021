#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor arm_motorsMotorA = motor(PORT3, ratio18_1, false);
motor arm_motorsMotorB = motor(PORT8, ratio18_1, true);
motor_group arm_motors = motor_group(arm_motorsMotorA, arm_motorsMotorB);
motor left_motor = motor(PORT1, ratio18_1, false);
motor right_motor = motor(PORT2, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}