/// test file no compeition flags
#include "vex.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <cmath>

using namespace vex;

brain Brain;

motor LMoveMotorA = motor(PORT14, ratio6_1, false);
motor LMoveMotorB = motor(PORT12, ratio6_1, false);
motor_group LMove = motor_group(LMoveMotorA, LMoveMotorB);
motor RMoveMotorA = motor(PORT11, ratio6_1, true);
motor RMoveMotorB = motor(PORT13, ratio6_1, true);
motor_group RMove = motor_group(RMoveMotorA, RMoveMotorB);
drivetrain Drivetrain = drivetrain(LMove, RMove, 319.024, 284.48, 196.85, mm, 1);

controller Controller1 = controller(primary);
motor Intake = motor(PORT5, ratio6_1, true);
digital_out Loader = digital_out(Brain.ThreeWirePort.H);
inertial Gyro = inertial(PORT6);
gps GPS = gps(PORT7, 88.9, -177.8, mm, 180);
optical colorSensor = optical(PORT1);

int heading_deg = 0, Xmove_distance = 0, Ymove_distance = 0;


void GPS_TurnToHeading(float Heading) {
  heading_deg = Heading;
  do {
    if (GPS.heading(deg) < heading_deg - 30) {
      LMove.spin(fwd, 15, pct);
      RMove.spin(fwd, -15, pct);
    } else if (GPS.heading(deg) > heading_deg + 30) {
      LMove.spin(fwd, -15, pct);
      RMove.spin(fwd, 15, pct);
    }
    if (GPS.heading(deg) < heading_deg - 12 && GPS.heading(deg) > heading_deg - 30) {
      LMove.spin(fwd, 5, pct);
      RMove.spin(fwd, -5, pct);
    } else if (GPS.heading(deg) > heading_deg + 12 && GPS.heading(deg) < heading_deg + 30) {
      LMove.spin(fwd, -5, pct);
      RMove.spin(fwd, 5, pct);
    }
  } while (GPS.heading(deg) > heading_deg + 12 || GPS.heading(deg) < heading_deg - 12);
  LMove.stop(hold);
  RMove.stop(hold);
  wait(0.5, msec);
}

void GPS_XMove(int Xdis) {
  Xmove_distance = Xdis;
  do {
    if (GPS.xPosition(mm) < Xmove_distance - 100) {
      LMove.spin(fwd, 20, pct);
      RMove.spin(fwd, 20, pct);
    } else if (GPS.xPosition(mm) > Xmove_distance + 100) {
      LMove.spin(fwd, -20, pct);
      RMove.spin(fwd, -20, pct);
    }
  } while (GPS.xPosition(mm) > Xmove_distance + 100 || GPS.xPosition(mm) < Xmove_distance - 100);
  LMove.stop(hold);
  RMove.stop(hold);
  wait(0.5, msec);
}

void GPS_YMove(int Ydis) {
  Ymove_distance = Ydis;
  do {
    if (GPS.yPosition(mm) < Ymove_distance - 100) {
      LMove.spin(fwd, -20, pct);
      RMove.spin(fwd, -20, pct);
    } else if (GPS.yPosition(mm) > Ymove_distance + 100) {
      LMove.spin(fwd, 20, pct);
      RMove.spin(fwd, 20, pct);
    }
  } while (GPS.yPosition(mm) > Ymove_distance + 100 ||
           GPS.yPosition(mm) < Ymove_distance - 100);
  LMove.stop(hold);
  RMove.stop(hold);
  wait(0.5, msec);
}

/*---------------------------------------------------------------------------*/
/*                          Shooting Functions                               */
/*---------------------------------------------------------------------------*/

bool isBlue(){
  return (colorSensor.hue() > 160 && colorSensor.hue() < 240);
}

void RedLG_Shoot(int shooter_time) {
  colorSensor.setLight(ledState::on);
  int shooter_time_now = 0;
  LMove.spin(fwd, -10, pct);
  RMove.spin(fwd, -10, pct);
  while (shooter_time_now <= shooter_time / 20) {
    if (!isBlue()) {
      Intake.spin(fwd, 100, pct);
    } else {
      Intake.spin(fwd, -100, pct);
    }
    shooter_time_now += 1;
    wait(20, msec);
  }
  Intake.stop();
  LMove.stop();
  RMove.stop();

  LMove.spin(fwd, 30, pct);
  RMove.spin(fwd, 30, pct);
  wait(600, msec);
  GPS_TurnToHeading(170);
  LMove.spin(fwd, 50, pct);
  RMove.spin(fwd, 50, pct);
  wait(500, msec);
  GPS_TurnToHeading(80);
  LMove.stop(brake);
  RMove.stop(brake);
}

void RedLeftShoot() {
  GPS_TurnToHeading(90);
  GPS_XMove(-340);
  GPS_TurnToHeading(180);
  GPS_YMove(420);
  GPS_TurnToHeading(316);

  // drive into goal
  LMove.spin(fwd, -50, pct);
  RMove.spin(fwd, -50, pct);
  wait(0.5, sec);

  // shoot while holding against goal
  Intake.spin(fwd, 100, pct);
  LMove.spin(fwd, -10, pct);
  RMove.spin(fwd, -10, pct);
  wait(3, sec);

  // Stop all motors
  Intake.stop(coast);
  LMove.stop(brake);
  RMove.stop(brake);
}

void auto_Isolation(void) {
  Brain.Screen.print("Isolation Phase");
  RedLeftShoot();
  RedLG_Shoot(3000);
}

int main() {
  Brain.Screen.clearScreen();

  Gyro.calibrate();
  while (Gyro.isCalibrating()) wait(50, msec);

  GPS.calibrate();
  while (GPS.isCalibrating()) wait(50, msec);
  GPS.setRotation(0, deg);

  Brain.Screen.print("Calibration done. Press screen to run.");

  // wait for screen press to start
  while (!Brain.Screen.pressing()) {
    wait(50, msec);
  }
  Brain.Screen.clearScreen();

  //  the test
  auto_Isolation();

  Brain.Screen.clearScreen();
  Brain.Screen.print("Test complete.");

  while (true) {
    wait(100, msec);
  }
}
