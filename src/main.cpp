/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  VEX AI Competition - Tracking and GPS Program             */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <cmath>

using namespace vex;

/// @brief i think we need a competition instance
competition Competition;
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


/// @brief the desired is important
std::string desired_detection = "red";
double desired_x = 317;
double x_tolerance = 50;
double min_y = 80;
double max_y = 400;
double forward_speed = 100;
double turn_speed = 10;
double Intake_speed = 400;
bool tracking_complete = false;
int track_cycle = 0;
const int MAX_TRACK_CYCLE = 3;

// Stuck 
bool is_tracking_stuck = false;
uint32_t last_active_time = 0;
const uint32_t STUCK_TIMEOUT = 5000;
double last_gps_x = 0.0;
double last_gps_y = 0.0;
const double POS_CHANGE_THRESHOLD = 100.0;

int heading_deg = 0, Xmove_distance = 0, Ymove_distance = 0;

// global serial port handle
FILE *serial_fp = NULL;


/// utility funcs
std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    tokens.push_back(item);
  }
  return tokens;
}

bool checkIfStuck() {
  if (is_tracking_stuck) return true;

  uint32_t current_time = Brain.timer(msec);
  double current_gps_x = GPS.xPosition(mm);
  double current_gps_y = GPS.yPosition(mm);
  double pos_change = sqrt(pow(current_gps_x - last_gps_x, 2) +
                           pow(current_gps_y - last_gps_y, 2)); // idrk what this math does but its in the other code

  if (pos_change > POS_CHANGE_THRESHOLD) {
    last_active_time = current_time;
    last_gps_x = current_gps_x;
    last_gps_y = current_gps_y;
    return false;
  }

  if (current_time - last_active_time > STUCK_TIMEOUT) {
    is_tracking_stuck = true;
    Brain.Screen.clearLine(7);
    Brain.Screen.print("STUCK DETECTED! Timeout: %dms", STUCK_TIMEOUT);
    Drivetrain.stop();
    return true;
  }

  return false;
}


void handleCommand(const std::string &cmd) {
  
  
  if (track_cycle >= MAX_TRACK_CYCLE || is_tracking_stuck) return;
  
  /// he left undefined behavior if buffer is empyu
  std::vector<std::string> parts = split(cmd, ',');

  if(parts.empty()) return;
  ///right here if parts at 0 is empty it will be undefined and break
  std::string detection_type = parts[0];
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Recv cmd: %s | Cycle: %d/%d",
                     cmd.c_str(), track_cycle + 1, MAX_TRACK_CYCLE);

  // if we completed a track and see the target again, restart tracking
  if (detection_type == desired_detection && tracking_complete) {
    tracking_complete = false;
    Brain.Screen.clearLine(4);
    Brain.Screen.print("Found new target! Restart tracking...");
  }

  if (tracking_complete) return;

  // No target detected 
  if (detection_type == "none") {
    Drivetrain.stop();
    Intake.stop();
    return;
  }

  // Process desired target type
  if (detection_type == desired_detection && parts.size() >= 3) {
    Intake.spin(forward, Intake_speed, rpm);
    double current_x = std::atof(parts[1].c_str());
    double current_y = std::atof(parts[2].c_str());
    double x_error = desired_x - current_x;

    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f", current_x, current_y);

    if (current_y >= min_y && current_y <= max_y) {
      if (std::fabs(x_error) <= x_tolerance) {
        // Aligned - drive forward
        Drivetrain.drive(forward, forward_speed, rpm);
      } else {
        // Not aligned - point turn to adjust
        const int turn_pct = 5;
        if (x_error > 0) {
          LMove.spin(fwd, -turn_pct, pct);
          RMove.spin(fwd, turn_pct, pct);
        } else {
          LMove.spin(fwd, turn_pct, pct);
          RMove.spin(fwd, -turn_pct, pct);
        }
      }
    } else if (current_y > max_y) {
      // target very close slow forward then complete this cycle
      Drivetrain.drive(forward, forward_speed / 2.0, rpm);
      wait(1500, msec);
      Drivetrain.stop();
      Intake.stop();
      tracking_complete = true;
      track_cycle++;

      last_active_time = Brain.timer(msec);
      last_gps_x = GPS.xPosition(mm);
      last_gps_y = GPS.yPosition(mm);

      Brain.Screen.clearLine(3);
      Brain.Screen.print("Track done! Total: %d/%d", track_cycle, MAX_TRACK_CYCLE);

      if (track_cycle < MAX_TRACK_CYCLE) {
        Brain.Screen.clearLine(4);
        Brain.Screen.print("Searching for next target...");
      }
    } else {
      // gtarget too far - move faster
      Drivetrain.drive(forward, forward_speed * 1.5, rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Moving closer...");
    }
  } else {
    // Wrong target type - stop
    Intake.stop();
    Drivetrain.stop();
  }
}
void GPS_TurnToHeading(float Heading) {
  heading_deg = Heading;
  double current;
  do {
    current = GPS.heading(deg); // in the future make the kids replace all of them and teach them importance of local vars
    if (current < heading_deg - 30) {
      LMove.spin(fwd, 15, pct);
      RMove.spin(fwd, -15, pct);
    } else if (current > heading_deg + 30) {
      LMove.spin(fwd, -15, pct);
      RMove.spin(fwd, 15, pct);
    }
    if (current < heading_deg - 12 && current > heading_deg - 30) {
      LMove.spin(fwd, 5, pct);
      RMove.spin(fwd, -5, pct);
    } else if (current> heading_deg + 12 && current < heading_deg + 30) {
      LMove.spin(fwd, -5, pct);
      RMove.spin(fwd, 5, pct);
    }
  } while (GPS.heading(deg  ) > heading_deg + 12 || GPS.heading(deg) < heading_deg - 12);
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
/// amys helper funcition idk if its the right values though
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

void blueLG_Shoot(int shoot_time){
  colorSensor.setLight(ledState::on);
  int shooter_time_now = 0;
  GPS_TurnToHeading(90);
  GPS_XMove(630);
  GPS_TurnToHeading(180);
  GPS_YMove(-1200);
  GPS_TurnToHeading(90);

  // drive into goal
  
  LMove.spin(fwd, -10, pct);
  RMove.spin(fwd, -10, pct);
  wait(0.5, sec);
  Intake.spin(fwd, 100, pct);
  wait(3, sec); 
}


void RedLeftShoot() {
  GPS_TurnToHeading(90);
  GPS_XMove(-340);
  GPS_TurnToHeading(180);
  GPS_YMove(420);
  GPS_TurnToHeading(316);

  // Drive into goal
  LMove.spin(fwd, -50, pct);
  RMove.spin(fwd, -50, pct);
  wait(0.5, sec);

  // Shoot while holding against goal
  Intake.spin(fwd, 100, pct);
  LMove.spin(fwd, -10, pct);
  RMove.spin(fwd, -10, pct);
  wait(5, sec);

  // Stop all motors
  Intake.stop(coast);
  LMove.stop(brake);
  RMove.stop(brake);
}

void pre_autonomous(void) {
  Gyro.calibrate();
  while (Gyro.isCalibrating()) wait(50, msec);

  GPS.calibrate();
  while (GPS.isCalibrating()) wait(50, msec);
  GPS.setRotation(0, deg);

  last_active_time = Brain.timer(msec);
  last_gps_x = GPS.xPosition(mm);
  last_gps_y = GPS.yPosition(mm);

  serial_fp = fopen("/dev/serial1", "r");
  if (!serial_fp) {
    Brain.Screen.print("Failed to open serial1!");
  }
}


void runTrackingAndGPSCycle() {

  if (serial_fp == NULL) {
    Brain.Screen.print("Serial port not open!");
    return;
  }

  // Reset tracking state
  //while (global_cycle)
  // if this code works plz add a global cycle so the robot will continue search
  // or maybe impliment an mdp of sorts (can compute offline and use a lookup table cuz vex brain is weak)
  track_cycle = 0;
  tracking_complete = false;
  is_tracking_stuck = false;
  last_active_time = Brain.timer(msec);
  last_gps_x = GPS.xPosition(mm);
  last_gps_y = GPS.yPosition(mm);

  Brain.Screen.clearLine(5);
  Brain.Screen.print("Starting tracking...");

  // Phase 1: spin to find initial target
  bool found_target = false;
  Drivetrain.turn(right, turn_speed * 1.5, rpm);

  while (!found_target && !checkIfStuck()) {
    char buffer[128] = {0};
    if (fgets(buffer, sizeof(buffer), serial_fp) != NULL) {
      size_t len = std::strlen(buffer);
      if (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
        buffer[len - 1] = '\0';
      }
      std::string cmd_str(buffer);
      std::vector<std::string> parts = split(cmd_str, ',');

      if (parts.size() > 0 && parts[0] == desired_detection) {
        found_target = true;
        Drivetrain.stop();
        Brain.Screen.clearLine(6);
        Brain.Screen.print("Found target! Starting tracking...");
        last_active_time = Brain.timer(msec);
        last_gps_x = GPS.xPosition(mm);
        last_gps_y = GPS.yPosition(mm);
      }
    }
    vex::this_thread::sleep_for(2);
  }

  // Phase 2: Track targets for MAX_TRACK_CYCLE cycles
  while (track_cycle < MAX_TRACK_CYCLE && !checkIfStuck()) {
    char buffer[128] = {0};
    if (fgets(buffer, sizeof(buffer), serial_fp) != NULL) {
      size_t len = std::strlen(buffer);
      if (len > 0 && (buffer[len - 1] == '\n' || buffer[len - 1] == '\r')) {
        buffer[len - 1] = '\0';
      }
      std::string cmd_str(buffer);
      handleCommand(cmd_str);
    }
    vex::this_thread::sleep_for(2);
  }

  // Phase 3: GPS navigation and shooting
  if (!is_tracking_stuck) {
    Brain.Screen.clearLine(5);
    Brain.Screen.print("Tracking done! Starting GPS...");
    wait(2, sec);
    RedLeftShoot();
  } else {
    Brain.Screen.clearLine(5);
    Brain.Screen.print("Stuck detected! Skipping GPS...");
    is_tracking_stuck = false;
  }

}

void auto_Isolation(void) {
  Brain.Screen.print("Isolation Phase");
  RedLeftShoot();

  // we need to move it to the right spot before we lgshoot
  RedLG_Shoot(3000);
}

void auto_Interaction(void) {
  Brain.Screen.print("Interaction Phase");
  runTrackingAndGPSCycle();
  RedLG_Shoot(5000);

}
/// idrk what this does but the other guy has it 
bool firstAutoFlag = true;

void autonomousMain(void) {
  if (firstAutoFlag)
    auto_Isolation();
  else
    auto_Interaction();

  firstAutoFlag = false;
}

int main() {
  
  pre_autonomous();

  Competition.autonomous(autonomousMain);

  while (true) {
    wait(100, msec);
  }
}
