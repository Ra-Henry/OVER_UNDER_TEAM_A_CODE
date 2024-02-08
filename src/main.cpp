/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       zhongshunrao                                              */
/*    Created:      10/15/2023, 1:12:41 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
motor RFMot(PORT8, false);
motor RMMot(PORT9, false);
motor RBMot(PORT10, false);
motor LFMot(PORT18, true);
motor LBMot(PORT19, true);
motor LMMot(PORT20,true);
motor_group rightDrive(RFMot, RMMot, RBMot);
motor_group leftDrive(LFMot, LMMot, LBMot);
motor_group drive(LFMot, LMMot, LBMot, RFMot, RMMot, RBMot);
motor flywheel(PORT11, true);
motor arm(PORT1, false);
motor intake(PORT13, false);
digital_out Wings1(Brain.ThreeWirePort.G);
digital_out Wings2(Brain.ThreeWirePort.H);
controller control(primary);
void move (int pos, int speed, bool stopping){
  drive.spinFor(pos, rotationUnits::deg, speed, velocityUnits::pct);
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
void pre_auton(void){}

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
move (-900, 100, true);
move (600, 100, true);
  //drive.spinToPosition (300, vex::deg, true);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

void drive_mV(int lSpeed,int rSpeed){
  leftDrive.spin(forward,lSpeed,voltageUnits::mV);
  rightDrive.spin(forward,rSpeed,voltageUnits::mV);
}

int isFlywheelSpin=0;
void toggleFlywheel(){
  isFlywheelSpin+=1;
}
\
int isWingOut= 0;
void toggleWings(){
  isWingOut +=1;
}

void usercontrol(void) {
  // User control code here, inside the loop
  control.ButtonY.pressed(toggleFlywheel);


  while (1) {
    double averageDriveTrainTemperature=leftDrive.temperature(celsius)+rightDrive.temperature(celsius);
    averageDriveTrainTemperature/=2;
    Brain.Screen.printAt(30,30,"Average Drivetrain Temperature: %.3f         ",averageDriveTrainTemperature);
    if(averageDriveTrainTemperature>60) Brain.Screen.drawRectangle(100,100,40,40,red);
    else Brain.Screen.drawRectangle(100,100,40,40,green);
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    leftDrive.setStopping(brake);
    rightDrive.setStopping(brake);
    //Drive
    int leftDrive_mV,rightDrive_mV;
    leftDrive_mV=rightDrive_mV=control.Axis3.position();
    leftDrive_mV+=control.Axis1.position();
    rightDrive_mV-=control.Axis1.position();
    leftDrive_mV*=120;
    rightDrive_mV*=120;
    drive_mV(leftDrive_mV,rightDrive_mV);
    // Puncher
    if(isFlywheelSpin%2==1) flywheel.spin(forward,11,volt);
    else flywheel.spin(forward,0,volt);

    // Arm
    if(control.ButtonL1.pressing()){
      arm.spin(forward,12,volt);
    }else if(control.ButtonL2.pressing()){
      arm.spin(reverse,12,volt);
    }else{
      arm.spin(reverse,0,volt);
      arm.setBrake(brake);
    }
    // Intake
    if(control.ButtonR1.pressing()){
      intake.spin(reverse,12,volt);
    }else if(control.ButtonR2.pressing()){
      intake.spin(forward,12,volt);
    }else{
      intake.spin(reverse,0,volt);
    }

    //Wings
    control.ButtonL1.pressed(toggleWings);
    if(isWingOut){ Wings1.set(true); Wings2.set(true);}
    else{ Wings1.set(false); Wings2.set(false);}

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
