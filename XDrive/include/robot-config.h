using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor DriveFL;
extern motor DriveFR;
extern motor DriveBL;
extern motor Lift;
extern motor Intake1;
extern motor Intake2;
extern motor Discarder;
extern motor DriveBR;
extern sonar US_top;
extern sonar US_vision;
extern optical Viss;
extern sonar US_out;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );