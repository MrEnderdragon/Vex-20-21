#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor DriveFL = motor(PORT1, ratio18_1, false);
motor DriveFR = motor(PORT2, ratio18_1, true);
motor DriveBL = motor(PORT3, ratio18_1, false);
motor Lift = motor(PORT7, ratio18_1, true);
motor Intake1 = motor(PORT13, ratio18_1, false);
motor Intake2 = motor(PORT14, ratio18_1, true);
motor Discarder = motor(PORT9, ratio18_1, false);
motor DriveBR = motor(PORT4, ratio18_1, true);
sonar US_top = sonar(Brain.ThreeWirePort.E);
sonar US_vision = sonar(Brain.ThreeWirePort.A);
optical Viss = optical(PORT11);
sonar US_out = sonar(Brain.ThreeWirePort.C);

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