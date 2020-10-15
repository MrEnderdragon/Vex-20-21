using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor Drive1;
extern motor Drive2;
extern motor DriveMid;
extern motor Lift;
extern motor Intake1;
extern motor Intake2;
extern signature Viss__RED_BALL;
extern signature Viss__BLUE_BALL;
extern signature Viss__SIG_3;
extern signature Viss__SIG_4;
extern signature Viss__SIG_5;
extern signature Viss__SIG_6;
extern signature Viss__SIG_7;
extern vision Viss;
extern motor Discarder;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );