/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       henryzhao                                                 */
/*    Created:      Fri Aug 28 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drive1               motor         1               
// Drive2               motor         2               
// DriveMid             motor         3               
// Lift                 motor         8               
// Intake1              motor         5               
// Intake2              motor         6               
// Viss                 vision        11              
// Discarder            motor         9               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
competition Competition;

# define mPi 3.14159265358979323846;

int driveThreshold = 10;
int turnThreshold = 10;

float liftAccelVal = 200;
float liftAccelSpeed = 0.0006;

float slowMult = 0.8;
float normMult = 1.9;
float multiplier = normMult;

bool slowmode = false;
int isBlue = -1;

double wheelDiam = 3.5;
double chassisWidth = 12.75;

// ---- FORWARD DRIVE MATH ----

double wheelCirc = wheelDiam * mPi;
double distPerWheelDeg =  wheelCirc / 360;

// ---- TURN DRIVE MATH ----

double turnCirc = chassisWidth * mPi;
double distPerTurnDeg = turnCirc / 360;
double wheelDegPerTurnDeg = distPerTurnDeg/distPerWheelDeg;

int controlAxis (int axis){
  switch(axis){
    case 1: {
      return Controller1.Axis1.position();
    }
    case 2: {
      return Controller1.Axis2.position();
    }
    case 3: {
      return Controller1.Axis3.position();
    }
    case 4: {
      return Controller1.Axis4.position();
    }

    default:
      return 0;
  }
}

bool controlButton (char button, bool top = false){
  switch(button){
    case 'A':{
      return Controller1.ButtonA.pressing();
    }
    case 'B':{
      return Controller1.ButtonB.pressing();
    }
    case 'X':{
      return Controller1.ButtonX.pressing();
    }
    case 'Y':{
      return Controller1.ButtonY.pressing();
    }

    case 'U':{
      return Controller1.ButtonUp.pressing();
    }
    case 'D':{
      return Controller1.ButtonDown.pressing();
    }
    case 'L':{
      return Controller1.ButtonLeft.pressing();
    }
    case 'R':{
      return Controller1.ButtonRight.pressing();
    }

    case 'l':{
      if(top) return Controller1.ButtonL1.pressing();
      else return Controller1.ButtonL2.pressing();
    }
    case 'r':{
      if(top) return Controller1.ButtonR1.pressing();
      else return Controller1.ButtonR2.pressing();
    }

    default:
      return false;
  }
}

void stopDrive(int motorr){
  //stops driving

  switch(motorr){
    case 1: Drive1.stop();
    case 2: Drive2.stop();
    case 3: DriveMid.stop();
  }
}

void drive(int speed1, int speed2, int speed3){
  //drive the robot with speed 1 & 2

  if(speed1 == 0) stopDrive(1);
  else Drive1.spin(vex::directionType::fwd,fmin(fmax(speed1, -100), 100) * multiplier,vex::velocityUnits::rpm);
  
  if(speed2 == 0) stopDrive(2);
  else Drive2.spin(vex::directionType::fwd,fmin(fmax(speed2, -100), 100) * multiplier,vex::velocityUnits::rpm);

  if(speed3 == 0) stopDrive(3);
  else DriveMid.spin(vex::directionType::fwd,fmin(fmax(speed3, -100), 100) * multiplier,vex::velocityUnits::rpm);
}

void driveDist(int speed, int dist){
  Drive1.spinFor(dist/distPerWheelDeg, vex::rotationUnits::deg,speed, vex::velocityUnits::rpm, false);
  Drive2.spinFor(dist/distPerWheelDeg, vex::rotationUnits::deg,speed, vex::velocityUnits::rpm);
}

void turnDegsCW(int speed, int degs){
  Drive1.spinFor(degs*wheelDegPerTurnDeg, vex::rotationUnits::deg,speed, vex::velocityUnits::rpm, false);
  Drive2.spinFor(-degs*wheelDegPerTurnDeg, vex::rotationUnits::deg, speed, vex::velocityUnits::rpm);
}

void initialize () {
  int origRed1[] = {10,10}; 
  int origBlue1[] = {270,10}; 

  Viss.setLedMode(vex::vision::ledMode::manual);
  Viss.setLedBrightness(100);

  int size[] = {200,100};

  Brain.Screen.setPenColor("#ff3700");
  Brain.Screen.drawRectangle(origRed1[0], origRed1[1], size[0], size[1], "#8c1e00");

  Brain.Screen.setPenColor("#00b7ff");
  Brain.Screen.drawRectangle(origBlue1[0], origBlue1[1], size[0], size[1], "#00719e");

  while(true){
    if(Brain.Screen.pressing() || controlButton('X') || controlButton('Y')){
      int X = Brain.Screen.xPosition();//X pos of press
      int Y = Brain.Screen.yPosition();// Y pos of press

      //Checks if press is within boundaries of rectangle
      if (((X >= origRed1[0] && X <= origRed1[0] + size[0]) && (Y >= origRed1[1] && Y <= origRed1[1] + size[1])) || controlButton('X')){
        Brain.Screen.drawRectangle(0, 0, 500, 500, "#8c1e00");
        isBlue = 0;
        break;
      }

      if (((X >= origBlue1[0] && X <= origBlue1[0] + size[0]) && (Y >= origBlue1[1] && Y <= origBlue1[1] + size[1])) || controlButton('Y')){
        Brain.Screen.drawRectangle(0, 0, 500, 500, "#00719e");
        isBlue = 1;
        break;
      }
    }
  }
}

void driveLoop () {

  int driveAms[] = {0,0,0};

  if(controlAxis(3) > driveThreshold || controlAxis(3) < -driveThreshold){
    driveAms[0] += controlAxis(3);
    driveAms[1] += controlAxis(3);
  }

  if(controlAxis(1) > turnThreshold || controlAxis(1) < -turnThreshold){
    driveAms[0] += controlAxis(1);
    driveAms[1] -= controlAxis(1);
  }

  drive(driveAms[0],driveAms[1],driveAms[2]);

  bool revDisc = false;
  if(controlButton('U')) revDisc = true;

  Viss.takeSnapshot(Viss__BLUE_BALL);
  if(Viss.objectCount > 0) {
    if(isBlue == 0){
      revDisc = true;
    }
    Viss.setLedColor(0, 0, 255);
  }else {
    Viss.takeSnapshot(Viss__RED_BALL);
    if(Viss.objectCount > 0) {
      if(isBlue == 1){
        revDisc = true;
      }
      Viss.setLedColor(255, 0, 0);
    }else {
      Viss.setLedColor(255, 255, 255);
    }
  }

  //Viss.setLedColor(255, 255, 255);

  if(controlButton('l', true)){
    Lift.spin(vex::directionType::fwd,fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    Discarder.spin(vex::directionType::fwd,(revDisc? -1:1) * fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    if(liftAccelVal < 200) liftAccelVal += liftAccelSpeed;
  }else if(controlButton('l', false)){
    Lift.spin(vex::directionType::fwd,-fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    Discarder.spin(vex::directionType::fwd,(revDisc? 1:-1) * fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    if(liftAccelVal < 200) liftAccelVal += liftAccelSpeed;
  }else {
    Lift.stop();
    Discarder.stop();

    liftAccelVal = 200;
  }

  if(controlButton('r', true)){
    Intake1.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);

  }else if(controlButton('r', false)){
    Intake1.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);

  }else {
    Intake1.stop();
    Intake2.stop();
  }

  if(controlButton('A', true)){
    if(slowmode){
      multiplier = normMult;
    }else {
      multiplier = slowMult;
    }

    slowmode = !slowmode;

  }

  driveLoop();
}

void autonomous () {
  turnDegsCW(50,90);
  vexDelay(1000);
  turnDegsCW(50,45);
  vexDelay(1000);
  turnDegsCW(50,-180);
  vexDelay(1000);
  driveDist(50,10);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Set up callbacks for autonomous and driver control periods.
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(driveLoop);

  // Run the pre-autonomous function.

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);  initialize();

  }
}
