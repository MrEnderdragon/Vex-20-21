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
// DriveFL              motor         1               
// DriveFR              motor         2               
// DriveBL              motor         3               
// Lift                 motor         7               
// Intake1              motor         13              
// Intake2              motor         14              
// Discarder            motor         9               
// DriveBR              motor         4               
// US_top               sonar         E, F            
// US_vision            sonar         A, B            
// Viss                 optical       11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

struct asyncParams {
  int timeDelay;
  motor *toStop;
};

using namespace vex;
competition Competition;

# define mPi 3.14159265358979323846
# define tile 23.5

// ---- USER INPUT VARIABLES ----
int driveThreshold = 10;
int turnThreshold = 10;
double ballDetectThresh = 4;
double ballOutThresh = 4;
double moveStuckThresh = 0.01; // in degrees
double moveStuckReps = 50;
int moveAcc = 10; // in millisecs per rep
int revTime = 300;

float liftAccelVal = 200;
float liftAccelSpeed = 0.0006;

float slowMult = 0.8;
float normMult = 1.9;

double wheelDiam = 2.5; //inches
double chassisWidth = 18.5; //inches

int blueHue = 155; // 60 - 250
float blueThresh = 95;

int redHue = 20; // 0-40
float redThresh = 20;

// ---- PROGRAM VARIABLES ----

float multiplier = normMult;

bool slowmode = false;
bool aPressed = false;
// int isBlue = -1;
int isBlue = 0;
int returnState = -1;
long cycles = 0;

int revCounter = 0;
int stopCounter = 0;

// ---- FORWARD DRIVE MATH ----

double wheelCirc = wheelDiam * mPi;
double distPerWheelDeg =  wheelCirc / 360;

// ---- TURN DRIVE MATH ----

double turnCirc = chassisWidth * mPi;
double distPerTurnDeg = turnCirc / 360;
double wheelDegPerTurnDeg = distPerTurnDeg/distPerWheelDeg-1;

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

double visUS() {
  //Brain.Screen.clearScreen();
  //Brain.Screen.printAt(50, 50, "%lf", US_vision.distance(vex::distanceUnits::in));
  return US_vision.distance(vex::distanceUnits::in);
}

double topUS() {
  return US_top.distance(vex::distanceUnits::in);
}

double outUS() {
  return US_vision.distance(vex::distanceUnits::in);
}

int getViss () {
  int res = Viss.hue();

  if(res >= blueHue-blueThresh && res <= blueHue+blueThresh){
    return 1;
  }else if(res >= redHue-redThresh && res <= redHue+redThresh){
    return 0;
  }
  return -1;
}

void stopDrive(int motorr){
  //stops driving

  switch(motorr){
    case 1: DriveFL.stop();
    case 2: DriveFR.stop();
    case 3: DriveBL.stop();
    case 4: DriveBR.stop();
  }
}

void returnHome () {

}

void driveTime (int speed, double amTime, double ang){
  double radAng = ((ang+45)/360)*2*mPi;
  double xSpin = cos(radAng);
  double ySpin = sin(radAng);

  double timeToSpin = std::max(std::abs(xSpin),std::abs(ySpin))/speed;

  int start = vex::timer::system();

  DriveFL.spin(vex::directionType::fwd, xSpin/timeToSpin, vex::velocityUnits::rpm);
  DriveBR.spin(vex::directionType::fwd, xSpin/timeToSpin, vex::velocityUnits::rpm);

  DriveFR.spin(vex::directionType::fwd, ySpin/timeToSpin, vex::velocityUnits::rpm);
  DriveBL.spin(vex::directionType::fwd, ySpin/timeToSpin, vex::velocityUnits::rpm);

  while(vex::timer::system() - start <= amTime){
    vex::task::sleep(moveAcc);
  }

  DriveFL.stop();
  DriveFR.stop();
  DriveBL.stop();
  DriveBR.stop();
}

//ang goes counter-clockwise
int driveDist (int speed, double dist, double ang, bool returrn, int speedDeccel = -100){
  if(speedDeccel == -100){
    speedDeccel = speed;
  }
  double radAng = ((ang+45)/360)*2*mPi;
  double xSpin = dist*cos(radAng) / distPerWheelDeg;
  double ySpin = dist*sin(radAng) / distPerWheelDeg;

  double flRot = 0;
  double frRot = 0;
  double blRot = 0;
  double brRot = 0;

  int counter = 0;

  DriveFL.resetRotation();
  DriveFR.resetRotation();
  DriveBR.resetRotation();
  DriveBL.resetRotation();

  double timeToSpin = std::max(std::abs(xSpin),std::abs(ySpin))/speed;
  double timeToSpinSlow = std::max(std::abs(xSpin),std::abs(ySpin))/speedDeccel;
  bool isStopped = false;

  DriveFL.spin(vex::directionType::fwd, xSpin/timeToSpin, vex::velocityUnits::rpm);
  DriveBR.spin(vex::directionType::fwd, xSpin/timeToSpin, vex::velocityUnits::rpm);

  DriveFR.spin(vex::directionType::fwd, ySpin/timeToSpin, vex::velocityUnits::rpm);
  DriveBL.spin(vex::directionType::fwd, ySpin/timeToSpin, vex::velocityUnits::rpm);

  bool stopFL = false;
  bool stopFR = false;
  bool stopBL = false;
  bool stopBR = false;

  while(!isStopped){
    isStopped = true;

    if(std::abs(DriveFL.rotation(vex::rotationUnits::deg)) > std::abs(xSpin) || stopFL) {
      DriveFL.stop();
      stopFL = true;
    }else isStopped = false;

    if(std::abs(DriveBR.rotation(vex::rotationUnits::deg)) > std::abs(xSpin) || stopBR){
      DriveBR.stop();
      stopBR = true;
    }else isStopped = false;

    if(std::abs(DriveFR.rotation(vex::rotationUnits::deg)) > std::abs(ySpin) || stopFR) {
      DriveFR.stop();
      stopFR = true;
    }else isStopped = false;
    
    if(std::abs(DriveBL.rotation(vex::rotationUnits::deg)) > std::abs(ySpin) || stopBL) {
      DriveBL.stop();
      stopBL = true;
    }else isStopped = false;

    if((std::abs(DriveFL.rotation(vex::rotationUnits::deg)) > std::abs(xSpin)*2/3 && !stopFL) &&
    (std::abs(DriveBR.rotation(vex::rotationUnits::deg)) > std::abs(xSpin)*2/3 && !stopBR) && 
    (std::abs(DriveFR.rotation(vex::rotationUnits::deg)) > std::abs(ySpin)*2/3 && !stopFR) && 
    (std::abs(DriveBL.rotation(vex::rotationUnits::deg)) > std::abs(ySpin)*2/3 && !stopBL)) {
      DriveFL.spin(vex::directionType::fwd, xSpin/timeToSpinSlow, vex::velocityUnits::rpm);
      DriveBR.spin(vex::directionType::fwd, xSpin/timeToSpinSlow, vex::velocityUnits::rpm);

      DriveFR.spin(vex::directionType::fwd, ySpin/timeToSpinSlow, vex::velocityUnits::rpm);
      DriveBL.spin(vex::directionType::fwd, ySpin/timeToSpinSlow, vex::velocityUnits::rpm);
    }

    if((DriveFL.isSpinning() && (std::abs(DriveFL.rotation(vex::rotationUnits::deg)) - std::abs(flRot) <= moveStuckThresh)) || 
    (DriveFR.isSpinning() && (std::abs(DriveFR.rotation(vex::rotationUnits::deg)) - std::abs(frRot) <= moveStuckThresh)) || 
    (DriveBL.isSpinning() && (std::abs(DriveBL.rotation(vex::rotationUnits::deg)) - std::abs(blRot) <= moveStuckThresh)) || 
    (DriveBR.isSpinning() && (std::abs(DriveBR.rotation(vex::rotationUnits::deg)) - std::abs(brRot) <= moveStuckThresh))){
      counter++;
    }else {
      counter = 0;
      flRot = DriveFL.rotation(vex::rotationUnits::deg);
      frRot = DriveFR.rotation(vex::rotationUnits::deg);
      blRot = DriveBL.rotation(vex::rotationUnits::deg);
      brRot = DriveBR.rotation(vex::rotationUnits::deg);
    }

    if(counter > moveStuckReps){
      DriveFL.stop();
      DriveFR.stop();
      DriveBL.stop();
      DriveBR.stop();

      if(returrn) {
        returnHome();
      }

      break;
    }

    // flRot = DriveFL.rotation(vex::rotationUnits::deg);
    // frRot = DriveFR.rotation(vex::rotationUnits::deg);
    // blRot = DriveBL.rotation(vex::rotationUnits::deg);
    // brRot = DriveBR.rotation(vex::rotationUnits::deg);

    vex::task::sleep(moveAcc);
  }

  double xDist = DriveFL.rotation(vex::rotationUnits::deg) * distPerWheelDeg;
  double yDist = DriveBL.rotation(vex::rotationUnits::deg) * distPerWheelDeg;

  double distOut = sqrt(xDist*xDist + yDist*yDist);

  return distOut;
}

void turnDegsCW(double speed, double degs){
  DriveFL.spinFor(degs * wheelDegPerTurnDeg, vex::rotationUnits::deg, speed, vex::velocityUnits::rpm, false);
  DriveBL.spinFor(degs * wheelDegPerTurnDeg, vex::rotationUnits::deg, speed, vex::velocityUnits::rpm, false);

  DriveFR.spinFor(-degs * wheelDegPerTurnDeg, vex::rotationUnits::deg, speed, vex::velocityUnits::rpm, false);
  DriveBR.spinFor(-degs * wheelDegPerTurnDeg, vex::rotationUnits::deg, speed, vex::velocityUnits::rpm);
}

void drive(int speed1, int speed2, int speed3, int speed4){
  float res1 = fmax(fmin(speed1, 100),-100);
  float res2 = fmax(fmin(speed2, 100),-100);
  float res3 = fmax(fmin(speed3, 100),-100);
  float res4 = fmax(fmin(speed4, 100),-100);

  if(speed1 == 0) stopDrive(1);
  else DriveFL.spin(vex::directionType::fwd,res1 * multiplier,vex::velocityUnits::rpm);
  
  if(speed2 == 0) stopDrive(2);
  else DriveFR.spin(vex::directionType::fwd,res2 * multiplier,vex::velocityUnits::rpm);

  if(speed3 == 0) stopDrive(3);
  else DriveBL.spin(vex::directionType::fwd,res3 * multiplier,vex::velocityUnits::rpm);

  if(speed4 == 0) stopDrive(4);
  else DriveBR.spin(vex::directionType::fwd,res4 * multiplier,vex::velocityUnits::rpm);
}

void initialize () {
  int origRed1[] = {10,10}; 
  int origBlue1[] = {270,10}; 

  DriveFL.setBrake(vex::brakeType::brake);
  DriveFR.setBrake(vex::brakeType::brake);
  DriveBL.setBrake(vex::brakeType::brake);
  DriveBR.setBrake(vex::brakeType::brake);

  int size[] = {200,100};

  Brain.Screen.setPenColor("#ff3700");
  Brain.Screen.drawRectangle(origRed1[0], origRed1[1], size[0], size[1], "#8c1e00");

  Brain.Screen.setPenColor("#00b7ff");
  Brain.Screen.drawRectangle(origBlue1[0], origBlue1[1], size[0], size[1], "#00719e");

  while(true){
    if(Brain.Screen.pressing() || controlButton('X') || controlButton('Y') || controlButton('B')){
      int X = Brain.Screen.xPosition();//X pos of press
      int Y = Brain.Screen.yPosition();// Y pos of press

      if(controlButton('B')){
        int size[] = {200,100};

        Brain.Screen.setPenColor("#ff3700");
        Brain.Screen.drawRectangle(origRed1[0], origRed1[1], size[0], size[1], "#8c1e00");

        Brain.Screen.setPenColor("#00b7ff");
        Brain.Screen.drawRectangle(origBlue1[0], origBlue1[1], size[0], size[1], "#00719e");
        isBlue = -1;
      }

      //Checks if press is within boundaries of rectangle
      // red rect (X)
      if (controlButton('X') || ((X >= origRed1[0] && X <= origRed1[0] + size[0]) && (Y >= origRed1[1] && Y <= origRed1[1] + size[1]))){
        Brain.Screen.drawRectangle(0, 0, 500, 500, "#8c1e00");
        isBlue = 0;
        
        //blue rect (Y)
      }else if (controlButton('Y') || ((X >= origBlue1[0] && X <= origBlue1[0] + size[0]) && (Y >= origBlue1[1] && Y <= origBlue1[1] + size[1]))){
        Brain.Screen.drawRectangle(0, 0, 500, 500, "#00719e");
        isBlue = 1;
      }
    }

    vex::task::sleep(100);
  }
}

void driveLoop () {

  int driveAms[] = {0,0,0,0};

  // ---- DRIVING ----

  if(controlAxis(3) > driveThreshold || controlAxis(3) < -driveThreshold){
    driveAms[0] += controlAxis(3);
    driveAms[1] += controlAxis(3);
    driveAms[2] += controlAxis(3);
    driveAms[3] += controlAxis(3);
  }

  if(controlAxis(1) > turnThreshold || controlAxis(1) < -turnThreshold){
    driveAms[0] += controlAxis(1);
    driveAms[1] -= controlAxis(1);
    driveAms[2] += controlAxis(1);
    driveAms[3] -= controlAxis(1);
  }

  if(controlAxis(4) > turnThreshold || controlAxis(4) < -turnThreshold){
    driveAms[0] += controlAxis(4);
    driveAms[1] -= controlAxis(4);
    driveAms[2] -= controlAxis(4);
    driveAms[3] += controlAxis(4);
  }

  drive(driveAms[0],driveAms[1],driveAms[2],driveAms[3]);

  // ---- AUTO-SORT ----

  bool revDisc = false;
  // bool found = false;
  if(controlButton('U')) revDisc = true;

  // if(visUS() <= ballDetectThresh){
  //   Viss.takeSnapshot(Viss__BLUE_BALL);
  //   if(Viss.objectCount > 0) {
  //     if(isBlue == 0){
  //       revCounter = revTime;
  //       revDisc = true;
  //     }
  //     Viss.setLedColor(0, 0, 255);
  //   }else {
  //     Viss.takeSnapshot(Viss__RED_BALL);
  //     if(Viss.objectCount > 0) {
  //       if(isBlue == 1){
  //         revCounter = revTime;
  //         revDisc = true;
  //       }
  //       Viss.setLedColor(255, 0, 0);
  //     }else {
  //       Viss.setLedColor(255, 255, 255);
  //     }
  //   }
  // }

  if(visUS() <= ballDetectThresh){
    int visGot = getViss();

    if(visGot == 1){
      if(isBlue == 0){
        revCounter = revTime;
        revDisc = true;
      }
    }else if(visGot == 0){
      if(isBlue == 1){
        revCounter = revTime;
        revDisc = true;
      }
    }
  }

  if(controlButton('R')){
    revDisc = false;
  }
  // ---- ELEVATOR ----

  if(controlButton('l', true)){
    Lift.spin(vex::directionType::fwd,fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    if(liftAccelVal < 200) liftAccelVal += liftAccelSpeed;
  }else if(controlButton('l', false)){
    Lift.spin(vex::directionType::fwd,-fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    if(liftAccelVal < 200) liftAccelVal += liftAccelSpeed;
  }else {
    Lift.stop();

    liftAccelVal = 200;
  }

  if(controlButton('D')){
    Discarder.spin(vex::directionType::fwd,-1 * fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
  }else {
    if(controlButton('l', true)){
      Discarder.spin(vex::directionType::fwd,(revDisc? -0.8:1) * fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    }else if (controlButton('l', false)) {
      Discarder.spin(vex::directionType::fwd,(revDisc? 1:-0.8) * fmin(liftAccelVal, 200),vex::velocityUnits::rpm);
    } else {
      Discarder.stop();
    }
  }

  // ---- INTAKES ----

  if(controlButton('r', true)){
    Intake1.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);

  }else if(controlButton('r', false)){
    Intake1.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);

  }else {
    Intake1.stop();
    Intake2.stop();
  }

  // ---- SLOWMODE ----

  if(controlButton('A', true) && !aPressed){
    if(slowmode){
      multiplier = normMult;
      slowmode = false;
    }else {
      multiplier = slowMult;
      slowmode = true;
    }
    aPressed = true;
  }else if (!controlButton('A', true)){
    aPressed = false;
  }

  cycles++;

  driveLoop();
}

void expand () {
  Discarder.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  Intake1.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);

  vex::task::sleep(1000);

  driveDist(50, 5, 270, false);
  driveDist(50, 5, 90, false);
}

void cycleBallsInt (int ballsToCheck, int sleepTime){

  bool isAll = false;
  int lastBall = -1;

  Lift.spin(vex::directionType::fwd, 200,vex::velocityUnits::rpm);
  Discarder.spin(vex::directionType::fwd, 50,vex::velocityUnits::rpm);
  Intake1.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  vex::task::sleep(100);

  Discarder.spin(vex::directionType::rev, 200,vex::velocityUnits::rpm);
  vex::task::sleep(100);

  Discarder.spin(vex::directionType::fwd, 200,vex::velocityUnits::rpm);

  while(!isAll){
    isAll = true;

    for (int i = 0; i < ballsToCheck; i++){

      //wait...
      while(visUS() > ballDetectThresh) vex::task::sleep(50);
      vex::task::sleep(50);

      Controller1.rumble(".");

      //check colour
      int vissRes = getViss();
      if((lastBall == -1) || (lastBall == vissRes && vissRes != -1)){
        lastBall = vissRes;
      }else if ((lastBall != vissRes && vissRes != -1)){
        isAll = false;
      }

      //check sort reverse
      bool revDisc = true;

      if(vissRes == -1 || vissRes == isBlue){
        revDisc = false;
      }else {
        revDisc = true;
      }

      vex::task::sleep(250);

      //auto-sort
      Discarder.spin(revDisc ? vex::directionType::rev : vex::directionType::fwd, 175,vex::velocityUnits::rpm);
      vex::task::sleep(100);
      while(outUS() <= ballOutThresh) vex::task::sleep(10);
      Discarder.spin(vex::directionType::fwd, 175,vex::velocityUnits::rpm);
    }
  }
  
  Intake1.spin(vex::directionType::fwd, 100,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, 100,vex::velocityUnits::rpm);

  vex::task::sleep(sleepTime);
}

void cycleBallsDumb (int ballsRemove, int ballsIn, int sleepTime){
  vex::task::sleep(sleepTime);

  Lift.spin(vex::directionType::fwd, 200,vex::velocityUnits::rpm);
  Discarder.spin(vex::directionType::rev, 200,vex::velocityUnits::rpm);
  Intake1.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);

  for (int i = 0; i < ballsRemove; i++){
    //wait...
    while(visUS() > ballDetectThresh) vex::task::sleep(10);
    while(outUS() <= ballOutThresh) vex::task::sleep(10);
  }

  Intake1.stop();
  Intake2.stop();
  Discarder.spin(vex::directionType::fwd, 200,vex::velocityUnits::rpm);

  vex::task::sleep(sleepTime);
}

int asyncStop (void* args) {
  asyncParams *syncs = (asyncParams *) args;
  vex::task::sleep(syncs->timeDelay);
  syncs->toStop->stop();
  return 0;
}

void skillAutonomous() {
  expand();

  Discarder.stop();
  Lift.spin(vex::directionType::fwd, 50, vex::velocityUnits::rpm);
  Intake1.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 150,vex::velocityUnits::rpm);
  
  driveDist(150, tile+2, 0, false, 75);

  Lift.stop();

  turnDegsCW(100, -(90+7));
  // double driven = driveDist(150, tile, 0, false);
  driveDist(150, tile-4, 0, false, 75);

  

  Lift.spin(vex::directionType::fwd, 50, vex::velocityUnits::rpm);
  Discarder.spin(vex::directionType::fwd, 50,vex::velocityUnits::rpm);

  struct asyncParams asyncs;
  asyncs = {1000,&Discarder};
  vex::task t(asyncStop, (void*) &asyncs);

  struct asyncParams asyncs2;
  asyncs2 = {1000,&Lift};
  vex::task t2(asyncStop, (void*) &asyncs2);

  // driveDist(100, driven, 180, false);
  driveDist(100, tile-2, 180, false, 50);

  turnDegsCW(100, -45+0);
  driveDist(100, sqrt(2)*tile+2, 0, false, 50);

  // Lift.spin(vex::directionType::fwd, 100, vex::velocityUnits::rpm);
  // Discarder.spin(vex::directionType::fwd, 200,vex::velocityUnits::rpm);
  // vex::task::sleep(3000);

  cycleBallsInt(2, 3000);
  // cycleBallsDumb(2, 2, 3000);

  driveDist(100, sqrt(2)*tile-2, 180, false);

  Discarder.stop();

  Lift.spin(vex::directionType::fwd, 50, vex::velocityUnits::rpm);
  Intake1.spin(vex::directionType::rev, 200,vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 200,vex::velocityUnits::rpm);

  turnDegsCW(150, 90+45+10);

  //WALL ALIGN
  driveTime(100, 2000, 180);

  driveDist(100, tile+tile/2, 0, false, 50);

  turnDegsCW(150, 95);

  driveDist(100, tile/2, 0, false, 75);
  driveDist(50, tile, 0, false);

  turnDegsCW(100, -90);
  driveDist(100, tile/2, 0, false);

  // cycleBallsDumb(0, 1, 3000);

  driveDist(150, tile/2-1, 180, false);
  driveDist(100, tile/2-1, 0, false);
  driveDist(150, tile/2-1, 180, false);
  driveDist(100, tile/2-1, 0, false);
  driveDist(150, tile/2-1, 180, false);

  // turnDegsCW(100, -90);
  // driveDist(150, tile, 0, false);
  // turnDegsCW(150, 45);

  // driveDist(100, sqrt(2)*2*tile-3, 0, false);
  // cycleBallsInt(2, 3000);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Set up callbacks for autonomous and driver control periods.
  skillAutonomous();
  // Competition.autonomous(skillAutonomous);
  Competition.drivercontrol(driveLoop);

  // Run the pre-autonomous function.
  initialize();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
