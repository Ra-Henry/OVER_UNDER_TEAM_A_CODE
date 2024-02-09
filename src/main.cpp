/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       zhongshunrao                                              */
/*    Created:      10/15/2023, 1:12:41 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include<iostream>

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
motor LMMot(PORT20, true);
motor_group rightDrive(RFMot, RMMot, RBMot);
motor_group leftDrive(LFMot, LMMot, LBMot);
motor_group drive(LFMot, LMMot, LBMot, RFMot, RMMot, RBMot);
motor flywheel(PORT11, true);
motor arm(PORT1, false);
motor intake(PORT13, false);
digital_out Wings1(Brain.ThreeWirePort.G);
digital_out Wings2(Brain.ThreeWirePort.H);
controller control(primary);
inertial inertials(PORT14, turnType::right);

void move(int pos, int speed, bool stopping) {
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
void pre_auton(void) {

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void drive_mV(int lSpeed, int rSpeed) {
  leftDrive.spin(forward, lSpeed, voltageUnits::mV);
  rightDrive.spin(forward, rSpeed, voltageUnits::mV);
}

void drivePct(int lSPeed, int rSpeed) {
  drive_mV(lSPeed * 120, rSpeed * 120);
}

const double kp = 0.4, ki = 0.5, kd = 0, threshold = 10, speedPct = 100;

void drivePID(int dist, int timeout) {
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  double li, ri;
  double ld, rd;
  li = ri = ld = rd = 0;
  double lperr, lerr, rperr, rerr; // left prev_error, left error, etc.
  lperr = lerr = rperr = rerr = dist;
  double lcurr, rcurr; lcurr = rcurr = 0;
  double lvelo, rvelo;
  vex::timer t1;
  while (t1.time(vex::msec) < timeout) {
    vex::wait(10, vex::msec);
    lcurr = leftDrive.position(vex::deg);
    rcurr = rightDrive.position(vex::deg);
    lerr = dist - lcurr;
    rerr = dist - rcurr;
    std::cout << "PID Drive error = (" << lerr << ", " << rerr << ")\n\n";
    ld = lerr - lperr;
    rd = rerr - rperr;
    if (lerr < 0 != lperr < 0) {
      li = 0;
    }
    if (rerr < 0 != rperr < 0) {
      ri = 0;
    }
    lperr = lerr;
    rperr = rerr;
    if (fabs(lerr) < threshold) li += lerr;
    if (fabs(rerr) < threshold) ri += rerr;
    lvelo = lerr * kp + li * ki + ld * kd;
    rvelo = rerr * kp + ri * ki + rd * kd;
    if (lvelo > speedPct) lvelo = speedPct;
    if (rvelo > speedPct) rvelo = speedPct;
    if (lvelo < -speedPct) lvelo = -speedPct;
    if (rvelo < -speedPct) rvelo = -speedPct;
    drivePct(lvelo, rvelo);
  }
  drivePct(0, 0);
}

void arcDist(double lmult, double rmult, double dist, int timeout) {
  leftDrive.resetPosition();
  rightDrive.resetPosition();
  double pi, pd;
  pi = pd = 0;
  double perr, err;
  perr = err = dist;
  double curr = 0;
  double velo;
  vex::timer t1;
  while (t1.time(vex::msec) < timeout) {
    vex::wait(10, vex::msec);
    if (lmult > rmult) curr = leftDrive.position(vex::deg);
    else curr = rightDrive.position(vex::deg);
    err = dist - curr;
    std::cout << "PID arcDist error = " << curr << ")\n\n";
    pd = err - perr;
    if (err < 0 != perr < 0) {
      pi = 0;
    }
    perr = err;
    if (fabs(err) < threshold) pi += err;
    velo = err * kp + pi * ki + pd * kd;
    if (velo > speedPct) velo = speedPct;
    if (velo < -speedPct) velo = -speedPct;
    drivePct(velo * lmult, velo * rmult);
  }
  drivePct(0, 0);
}

double actual360 = 354.97;

double getRotation() {
  return inertials.rotation() / actual360 * 360;
}

void turnPID(double angle, int timeout, bool reset) {
  double threshold = 5, kp = 1, ki = 0.2, kd = 0.5, speedPct = 100;
  if (reset) inertials.resetRotation();
  double integral = 0;
  double derivative = 0;
  double prev_error, error;
  prev_error = error = angle;
  double curr = 0;
  double velo;
  vex::timer t1;
  while (t1.time(vex::msec) < timeout) {
    vex::wait(10, vex::msec);
    curr = getRotation();
    error = angle - curr;
    std::cout << "PID Turn error = " << error << "\n\n";
    derivative = error - prev_error;
    if (error < 0 != prev_error < 0) {
      integral = 0;
    }
    prev_error = error;
    if (fabs(error) < threshold) integral += error;
    velo = error * kp + integral * ki + derivative * kd;
    if (velo > speedPct) velo = speedPct;
    if (velo < -speedPct) velo = -speedPct;
    drivePct(velo, -velo);
  }
  drivePct(0, 0);
}

enum auton_mode {
  FAR_SIDE,
  CLOSE_SIDE,
  SKILLS,
  NONE,
};

auton_mode autonMode = SKILLS;

void handleBrainAutonSelect() {
  // x^2 + y^2 <= r^2
  int r = 60;
  r *= r;
  int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
  // Brain.Screen.printAt(30, 30, "%d     , %d     ", x, y);
  int X = 80, Y = 90;
  if (pow(x - X, 2) + pow(y - Y, 2) <= r) {
    autonMode = FAR_SIDE;
    Brain.Screen.printAt(30, 30, "Far    ");
    return;
  }
  X = 220;
  if (pow(x - X, 2) + pow(y - Y, 2) <= r) {
    autonMode = CLOSE_SIDE;
    Brain.Screen.printAt(30, 30, "Close    ");
    return;
  }
  X = 360;
  if (pow(x - X, 2) + pow(y - Y, 2) <= r) {
    autonMode = SKILLS;
    Brain.Screen.printAt(30, 30, "Skills    ");
    return;
  }
  autonMode = NONE;
  Brain.Screen.printAt(30, 30, "None    ");
}

void autonSelect() {
  // autonMode = NONE;
  Brain.Screen.printAt(30, 30, "None");
  Brain.Screen.drawCircle(80, 90, 50);
  Brain.Screen.printAt(50, 90, "Far");
  Brain.Screen.drawCircle(220, 90, 50);
  Brain.Screen.printAt(190, 90, "Close");
  Brain.Screen.drawCircle(360, 90, 50);
  Brain.Screen.printAt(330, 90, "Skills");
  Brain.Screen.pressed(handleBrainAutonSelect);
}

void farSideWP() {}

void closeSideWP() {}

void progSkills() {
  arcDist(0.5, 1, -550, 1000);
  drivePID(-400, 500);
  arcDist(-0.2, 1, 600, 1000);
  drivePID(-500, 1000);
  timer t1;
  t1.reset();
  while (t1.time(sec) < 50) {
    flywheel.spin(forward, 100, percent);
    wait(250, msec);
    flywheel.spin(forward, 99, percent);
    wait(250, msec);
  }
  flywheel.spin(forward, 0, volt);
  // turnPID(90, 20000, true);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  switch (autonMode) {
  case FAR_SIDE:
    farSideWP();
    break;
  case CLOSE_SIDE:
    closeSideWP();
    break;
  case SKILLS:
    progSkills();
    break;
  default:
    break;
  }
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

int isFlywheelSpin = 0;
void toggleFlywheel() {
  isFlywheelSpin += 1;
}

int isWingOut = 0;
void toggleWings() {
  isWingOut += 1;
}

void usercontrol(void) {
  // User control code here, inside the loop
  control.ButtonY.pressed(toggleFlywheel);


  while (1) {
    double averageDriveTrainTemperature = leftDrive.temperature(celsius) + rightDrive.temperature(celsius);
    averageDriveTrainTemperature /= 2;
    Brain.Screen.printAt(30, 30, "Average Drivetrain Temperature: %.3f         ", averageDriveTrainTemperature);
    if (averageDriveTrainTemperature > 60) Brain.Screen.drawRectangle(100, 100, 40, 40, red);
    else Brain.Screen.drawRectangle(100, 100, 40, 40, green);
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
    int leftDrive_mV, rightDrive_mV;
    leftDrive_mV = rightDrive_mV = control.Axis3.position();
    leftDrive_mV += control.Axis1.position();
    rightDrive_mV -= control.Axis1.position();
    leftDrive_mV *= 120;
    rightDrive_mV *= 120;
    drive_mV(leftDrive_mV, rightDrive_mV);
    // Puncher
    if (isFlywheelSpin % 2 == 1) flywheel.spin(forward, 11, volt);
    else flywheel.spin(forward, 0, volt);

    // Arm
    if (control.ButtonL1.pressing()) {
      arm.spin(forward, 12, volt);
    } else if (control.ButtonL2.pressing()) {
      arm.spin(reverse, 12, volt);
    } else {
      arm.spin(reverse, 0, volt);
      arm.setBrake(brake);
    }
    // Intake
    if (control.ButtonR1.pressing()) {
      intake.spin(reverse, 12, volt);
    } else if (control.ButtonR2.pressing()) {
      intake.spin(forward, 12, volt);
    } else {
      intake.spin(reverse, 0, volt);
    }

    //Wings
    control.ButtonL1.pressed(toggleWings);
    if (isWingOut) { Wings1.set(true); Wings2.set(true); } else { Wings1.set(false); Wings2.set(false); }

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

  autonSelect();

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
