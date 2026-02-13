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

std::string desired_detection = "blue";
double desired_x = 317;         // 目标x坐标（中心位置）
double x_tolerance = 50;        // x方向允许误差范围（像素）
double min_y = 80;             // 最小y坐标（过近阈值）
double max_y = 400;             // 最大y坐标（过远阈值）
double forward_speed = 200;     // 前进速度（rpm）
double turn_speed = 10;         // 转向速度（rpm）
double Intake_speed = 500;      // 3号电机转速（rpm）
double DownRoller_speed = 400;      // 5号电机转速（rpm）
bool tracking_complete = false; // 跟踪完成标志

// GPS相关变量
int heading_deg = 0, Xmove_distance = 0, Ymove_distance = 0;

// 字符串分割函数
std::vector<std::string> split(const std::string &s, char delim) 
{
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss, item, delim)) 
  {
    tokens.push_back(item);
  }
  return tokens;
}

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
 void handleCommand(const std::string &cmd) 
{
  if (tracking_complete) return; // 跟踪完成后不再处理命令
  
  std::vector<std::string> parts = split(cmd, ',');
  std::string detection_type = parts[0];
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Recv cmd: %s", cmd.c_str());

  // 如果未检测到任何目标，停止不动
  if (detection_type == "none") 
  {
    Drivetrain.stop();
    Intake.stop();
    return;
  }

  // 只处理期望的目标类型
  if(detection_type == desired_detection && parts.size() >= 3) 
  {
    Intake.spin(forward, Intake_speed, rpm);
    // DownRoller.spin(forward, DownRoller_speed, rpm);
    double current_x = std::atof(parts[1].c_str());
    double current_y = std::atof(parts[2].c_str());
    //desired = 317
    double x_error = desired_x - current_x;

    // 显示当前坐标信息
    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f", current_x, current_y);
    
    // 判断是否在有效y范围（目标距离合适）
    //min y =80 max y 400
    if (current_y >= min_y && current_y <= max_y) 
    {
      // 判断x方向是否在允许误差范围内
      if (std::fabs(x_error) <= x_tolerance) 
      {
        // x方向对齐，向前移动
        Drivetrain.drive(forward, forward_speed, rpm);
        
      } 
      else 
      { 
        Brain.Screen.clearLine(2);
        Brain.Screen.print("still in turning state");
        /**/
        // x方向未对齐，进行短时原地微调（每次微调后等待视觉更新）
        const int adjust_ms = 1000;     // 微调时长（ms）
        const int turn_pct = 5;       // 微调转速（%），可根据机器调整
        if (x_error > 0) {
          // 目标在右侧，向左原地转
          LMove.spin(forward, -turn_pct, pct);
          RMove.spin(forward, turn_pct, pct);
        } else {
          // 目标在左侧，向右原地转
          LMove.spin(forward, turn_pct, pct);
          RMove.spin(forward, -turn_pct, pct);
        }
        
      }
    } 
    else if (current_y > max_y) 
    {
      // y值过大（目标非常接近），减速向前直行1.5秒，然后立即停止并结束跟踪阶段
      Drivetrain.drive(forward, forward_speed/2.0, rpm);

      //Blocks any updates in the thread for 1.5 seconds.
      wait(1500, msec);
      Drivetrain.stop();
      Intake.stop();
      // DownRoller.stop();
      tracking_complete = true; // 结束跟踪阶段，退出主循环
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Close: slow forward 1.5s, tracking complete");
    } 
    else 
    {
      // 目标过远，先快速靠近
      Drivetrain.drive(forward, forward_speed * 1.5, rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Moving closer...");
    }
  }
  else
  {
    // 非目标类型，停止不动
    Intake.stop();
    Drivetrain.stop();
  }
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
  FILE *fp = fopen("/dev/serial1", "r");
  if(!fp) {
    Brain.Screen.print("Failed to open /dev/serial1");
    return;
  }
  Brain.Screen.print("Starting blue tracking...");

  // 第一阶段：执行blue跟踪程序
  while(!tracking_complete) 
  {
    char buffer[128] = {0};
    if(fgets(buffer, sizeof(buffer), fp) != NULL) 
    {
      size_t len = std::strlen(buffer);
      if(len>0 && (buffer[len-1]=='\n'||buffer[len-1]=='\r')) 
      {
        buffer[len-1] = '\0';
      }
      std::string cmd_str(buffer);
      handleCommand(cmd_str);
    }
    vex::this_thread::sleep_for(2);
  }

  // 跟踪完成后关闭串口
  fclose(fp);
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
  
  Brain.Screen.clearScreen();

  //  the test
  auto_Isolation();

  Brain.Screen.clearScreen();
  Brain.Screen.print("Test complete.");

  while (true) {
    wait(100, msec);
  }
}
