#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor front_arm_motorsMotorA = motor(PORT3, ratio18_1, true);
motor front_arm_motorsMotorB = motor(PORT4, ratio18_1, false);
motor_group front_arm_motors = motor_group(front_arm_motorsMotorA, front_arm_motorsMotorB);
motor left_motor = motor(PORT1, ratio18_1, false);
motor right_motor = motor(PORT2, ratio18_1, true);
motor back_arm_motorsMotorA = motor(PORT5, ratio18_1, false);
motor back_arm_motorsMotorB = motor(PORT6, ratio18_1, true);
motor_group back_arm_motors = motor_group(back_arm_motorsMotorA, back_arm_motorsMotorB);
motor left_small_motor = motor(PORT7, ratio18_1, false);
motor right_small_motor = motor(PORT8, ratio18_1, false);

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