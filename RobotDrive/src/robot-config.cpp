#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Drive1 = motor(PORT1, ratio18_1, false);
motor Drive2 = motor(PORT2, ratio18_1, true);
motor DriveMid = motor(PORT3, ratio18_1, false);
motor Lift = motor(PORT8, ratio18_1, true);
motor Intake1 = motor(PORT5, ratio18_1, false);
motor Intake2 = motor(PORT6, ratio18_1, true);
/*vex-vision-config:begin*/
signature Viss__RED_BALL = signature (1, 6525, 9349, 7938, -1513, -467, -990, 1.5, 0);
signature Viss__BLUE_BALL = signature (2, -2955, -1693, -2324, 5871, 10227, 8049, 2.5, 0);
signature Viss__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Viss__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Viss__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Viss__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Viss__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Viss = vision (PORT11, 70, Viss__RED_BALL, Viss__BLUE_BALL, Viss__SIG_3, Viss__SIG_4, Viss__SIG_5, Viss__SIG_6, Viss__SIG_7);
/*vex-vision-config:end*/
motor Discarder = motor(PORT9, ratio18_1, false);

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