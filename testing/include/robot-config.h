using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor_group arm_motors;
extern motor left_motor;
extern motor right_motor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );