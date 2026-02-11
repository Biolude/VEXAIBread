/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  VEX AI Competition Combined Tracking and GPS Program      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <cstdlib>  // atof
#include <cmath>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of competition
competition Competition;
brain  Brain;
bool RemoteControlCodeEnabled = true;

// 驱动电机配置
motor LMoveMotorA = motor(PORT11, ratio6_1, false);
motor LMoveMotorB = motor(PORT12, ratio6_1, false);
motor_group LMove = motor_group(LMoveMotorA, LMoveMotorB);
motor RMoveMotorA = motor(PORT19, ratio6_1, true);
motor RMoveMotorB = motor(PORT20, ratio6_1, true);
motor_group RMove = motor_group(RMoveMotorA, RMoveMotorB);
drivetrain Drivetrain = drivetrain(LMove, RMove, 259.34, 318, 254, mm, 1);

// 其他电机和传感器配置
controller Controller1 = controller(primary);
motor Intake = motor(PORT16, ratio6_1, false);
motor UpRoller = motor(PORT18, ratio18_1, true);
motor Shooter = motor(PORT6, ratio6_1, false);
motor DownRoller = motor(PORT17, ratio18_1, true);
optical Shooter_Optical = optical(PORT7);
digital_out Loader = digital_out(Brain.ThreeWirePort.H);
gps GPS = gps(PORT10, -87.50, -100.00, mm, 180);

// 跟踪配置参数
std::string desired_detection = "blue";
double desired_x = 310;         // 目标x坐标（中心位置）
double x_tolerance = 60;        // x方向允许误差范围（像素）
double target_y = 440;          // 目标y坐标阈值
double forward_speed = 250;     // 前进速度（rpm）
double turn_speed = 40;         // 转向速度（rpm）
double Intake_speed = 600;      // 3号电机转速（rpm）
double DownRoller_speed = 200;  // 5号电机转速（rpm）
bool tracking_complete = false; // 单次跟踪完成标志
int track_cycle = 0;            // 当前完成的跟踪循环次数
const int MAX_TRACK_CYCLE = 3;  // 总跟踪循环次数（三次）

// 卡死检测相关变量
bool is_tracking_stuck = false; // 卡死标志
uint32_t last_active_time = 0;  // 最后一次“有效活动”的时间戳（ms）
const uint32_t STUCK_TIMEOUT = 5000; // 卡死判定阈值（5秒）
double last_gps_x = 0.0;        // 上一次GPS X坐标
double last_gps_y = 0.0;        // 上一次GPS Y坐标
const double POS_CHANGE_THRESHOLD = 100.0; // 位置变化阈值（100mm）

// GPS相关变量
int heading_deg = 0, Xmove_distance = 0, Ymove_distance = 0;
// 全局串口句柄（方便复用函数访问）
FILE *serial_fp = NULL;

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

// 卡死检测函数
bool checkIfStuck() {
  if (is_tracking_stuck) return true;

  uint32_t current_time = Brain.timer(msec);
  double current_gps_x = GPS.xPosition(mm);
  double current_gps_y = GPS.yPosition(mm);

  double pos_change = sqrt(pow(current_gps_x - last_gps_x, 2) + pow(current_gps_y - last_gps_y, 2));

  if (pos_change > POS_CHANGE_THRESHOLD) {
    last_active_time = current_time;
    last_gps_x = current_gps_x;
    last_gps_y = current_gps_y;
    Brain.Screen.clearLine(7);
    Brain.Screen.print("Active! Pos change: %.1fmm", pos_change);
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

// 命令处理函数（跟踪逻辑）
void handleCommand(const std::string &cmd) 
{
  if (track_cycle >= MAX_TRACK_CYCLE || is_tracking_stuck) return;
  
  std::vector<std::string> parts = split(cmd, ',');
  std::string detection_type = parts[0];
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Recv cmd: %s | Cycle: %d/%d", cmd.c_str(), track_cycle + 1, MAX_TRACK_CYCLE);

  if (detection_type == desired_detection && tracking_complete) {
    tracking_complete = false;
    Brain.Screen.clearLine(4);
    Brain.Screen.print("Found new target! Restart tracking...");
  }

  if (tracking_complete) return;
  
  if (detection_type == "none") 
  {
    LMove.spin(reverse,turn_speed *1.5,rpm);
    RMove.spin(reverse,turn_speed *0.8,rpm);
    return;
  }

  if(detection_type == desired_detection && parts.size() >= 3) 
  {
    Intake.spin(forward, Intake_speed, rpm);
    DownRoller.spin(forward, DownRoller_speed, rpm);
    double current_x = std::atof(parts[1].c_str());
    double current_y = std::atof(parts[2].c_str());
    double x_error = desired_x - current_x;

    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f | Target: %s", current_x, current_y, desired_detection.c_str());

    if (current_y > 0 && current_y < target_y) 
    {
      if (std::fabs(x_error) <= x_tolerance) 
      {
        Drivetrain.drive(forward, forward_speed, rpm);
      } 
      else 
      {
        if (x_error > 0)
        {
          LMove.stop();
          RMove.spin(fwd,turn_speed,rpm);
        }
        else
        {
          RMove.stop();
          LMove.spin(fwd,turn_speed,rpm);
        }
      }
    } 
    else if (current_y >= target_y)  
    {
      Drivetrain.drive(forward, forward_speed, rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Y >= 430, Moving forward for 1s...");
  
      wait(3, sec);
      Drivetrain.stop();
      
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Waiting for 2s...");
      wait(2, sec);
      tracking_complete = true;
      track_cycle++;
      
      last_active_time = Brain.timer(msec);
      last_gps_x = GPS.xPosition(mm);
      last_gps_y = GPS.yPosition(mm);

      Brain.Screen.clearLine(3);
      Brain.Screen.print("Single track done! Total: %d/%d", track_cycle, MAX_TRACK_CYCLE);

      if (track_cycle < MAX_TRACK_CYCLE) 
      {
        Brain.Screen.clearLine(4);
        Brain.Screen.print("Spinning to find next %s...", desired_detection.c_str());
        LMove.spin(reverse,turn_speed *1.5,rpm);
        RMove.spin(reverse,turn_speed *0.5,rpm);
        wait(2, sec);
        
        Brain.Screen.clearLine(4);
        Brain.Screen.print("Waiting for next target...");
      }
    } 
    else  
    {
      LMove.spin(reverse,turn_speed *1.5,rpm);
      RMove.spin(reverse,turn_speed *0.5,rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Y <= 0, waiting for valid target...");
    }
  }
  else
  {
    LMove.spin(reverse,turn_speed *1.5,rpm);
    RMove.spin(reverse,turn_speed *0.5,rpm);
  }
}

/*---------------------------------------------------------------------------*/
/*                          GPS相关函数                                      */
/*---------------------------------------------------------------------------*/
void pre_autonomous(void) {
  
  GPS.calibrate();
  while(GPS.isCalibrating()) wait(50, msec);
  GPS.setRotation(0,deg);
  last_active_time = Brain.timer(msec);
  last_gps_x = GPS.xPosition(mm);
  last_gps_y = GPS.yPosition(mm);

  // 打开串口（AI竞赛模板中移到预自主阶段）
  serial_fp = fopen("/dev/serial1", "r");
  if(!serial_fp) {
    Brain.Screen.print("Failed to open serial1!");
  }
}

void GPS_TurnToHeading(float Heading)
{
  heading_deg = Heading;
  do{
      if(GPS.heading(deg) < heading_deg - 30)
      {
        LMove.spin(fwd,15,pct);
        RMove.spin(fwd,-15,pct);
      }
      else if(GPS.heading(deg) > heading_deg + 30)
      {
        LMove.spin(fwd,-15,pct);
        RMove.spin(fwd,15,pct);
      }
      if(GPS.heading(deg) < heading_deg - 12 && GPS.heading(deg) > heading_deg - 30)
      {
        LMove.spin(fwd,5,pct);
        RMove.spin(fwd,-5,pct);
      }
      else if(GPS.heading(deg) > heading_deg + 12 && GPS.heading(deg) < heading_deg + 30)
      {
        LMove.spin(fwd,-5,pct);
        RMove.spin(fwd,5,pct);
      }
    }while(GPS.heading(deg) > heading_deg + 12 || GPS.heading(deg) < heading_deg - 12);
    LMove.stop(brake);
    RMove.stop(brake);
    wait(0.5,msec);
}
void GPS_TurnToRotation(float Heading)
{
  heading_deg = Heading;
  do{
      if(GPS.rotation(deg) < heading_deg - 30)
      {
        LMove.spin(fwd,15,pct);
        RMove.spin(fwd,-15,pct);
      }
      else if(GPS.rotation(deg) > heading_deg + 30)
      {
        LMove.spin(fwd,-15,pct);
        RMove.spin(fwd,15,pct);
      }
      if(GPS.rotation(deg) < heading_deg - 12 && GPS.rotation(deg) > heading_deg - 30)
      {
        LMove.spin(fwd,5,pct);
        RMove.spin(fwd,-5,pct);
      }
      else if(GPS.rotation(deg) > heading_deg + 12 && GPS.rotation(deg) < heading_deg + 30)
      {
        LMove.spin(fwd,-5,pct);
        RMove.spin(fwd,5,pct);
      }
    }while(GPS.rotation(deg) > heading_deg + 12 || GPS.rotation(deg) < heading_deg - 12);
    LMove.stop(brake);
    RMove.stop(brake);
    wait(0.5,msec);
}
void GPS_XMove(int Xdis)
{
  Xmove_distance = Xdis;
  do{
      if(GPS.xPosition(mm)  < Xmove_distance - 100)
      {
        LMove.spin(fwd,20,pct);
        RMove.spin(fwd,20,pct);
      }
      else if(GPS.xPosition(mm)  > Xmove_distance + 100)
      {
        LMove.spin(fwd,-20,pct);
        RMove.spin(fwd,-20,pct);
      }
    }while(GPS.xPosition(mm)  > Xmove_distance + 100 || GPS.xPosition(mm) < Xmove_distance - 100);
    LMove.stop(brake);
    RMove.stop(brake);
    wait(0.5,msec);
}

void GPS_YMove(int Ydis)
{
  Ymove_distance = Ydis;
  do{
      if(GPS.yPosition(mm)  < Ymove_distance - 100)
      {
        LMove.spin(fwd,-20,pct);
        RMove.spin(fwd,-20,pct);
      }
      else if(GPS.yPosition(mm)  > Ymove_distance + 100)
      {
        LMove.spin(fwd,20,pct);
        RMove.spin(fwd,20,pct);
      }
    }while(GPS.yPosition(mm)  > Ymove_distance + 100 || GPS.yPosition(mm) < Ymove_distance - 100);
    LMove.stop(brake);
    RMove.stop(brake);
    wait(0.5,msec);
}
void GPS_YFUMove(int Ydis)
{
  Ymove_distance = Ydis;
  do{
      if(GPS.yPosition(mm)  < Ymove_distance - 100)
      {
        LMove.spin(fwd,20,pct);
        RMove.spin(fwd,20,pct);
      }
      else if(GPS.yPosition(mm)  > Ymove_distance + 100)
      {
        LMove.spin(fwd,-20,pct);
        RMove.spin(fwd,-20,pct);
      }
    }while(GPS.yPosition(mm)  > Ymove_distance + 100 || GPS.yPosition(mm) < Ymove_distance - 100);
    LMove.stop(brake);
    RMove.stop(brake);
    wait(0.5,msec);
}
void BlueLeftShoot()
{
  LMove.spin(fwd,-50,pct);
  RMove.spin(fwd,-50,pct);
  Intake.spin(fwd,-20,pct);
  DownRoller.spin(fwd,-20,pct);
  UpRoller.spin(fwd,-20,pct);
  wait(300,msec);
  Intake.stop();
  DownRoller.stop();
  UpRoller.stop();
  wait(200,msec);
  if(GPS.yPosition(mm) > 1300 or GPS.yPosition(mm) < -1300)
  {
    GPS_TurnToHeading(90);
    GPS_XMove(1200);
  }
  
  else if(GPS.xPosition(mm) < -700 and GPS.xPosition(mm) > -1200)
  {
    GPS_TurnToHeading(180);
    GPS_YMove(-600);
  }

  else if(GPS.xPosition(mm) > 1200)
  {
    GPS_TurnToHeading(90);
    GPS_XMove(-900);
    GPS_TurnToHeading(180);
    GPS_YMove(-600);
  }
  else if(GPS.xPosition(mm) > -700 && GPS.xPosition(mm) < 700)
  {
    if(GPS.yPosition(mm) > 0 && GPS.yPosition(mm) < 1000)
    {
      GPS_TurnToHeading(180);
      GPS_YMove(600);
    }
    else if(GPS.yPosition(mm) > -1000 && GPS.yPosition(mm) < 0)
    {
      GPS_TurnToHeading(180);
      GPS_YMove(-600);
    }
  }
  GPS_TurnToHeading(90);
  GPS_XMove(1200);
  GPS_TurnToHeading(180);
  GPS_YMove(-1300);
  GPS_TurnToHeading(90);
  LMove.spin(fwd,-50,pct);
  RMove.spin(fwd,-50,pct);
  wait(1,sec);
  Intake.spin(fwd,100,pct);
  DownRoller.spin(fwd,100,pct);
  UpRoller.spin(fwd,80,pct);
  Shooter.spin(fwd,100,pct);
  LMove.spin(fwd,-10,pct);
  RMove.spin(fwd,-10,pct);
  wait(5, sec);
  
  // 停止所有电机
  Intake.stop(coast);
  DownRoller.stop(coast);
  UpRoller.stop(coast);
  Shooter.stop(coast);
  LMove.stop(brake);
  RMove.stop(brake);
  Brain.Screen.clearLine(9);
    // ======================================
}
void BlueLG_Shoot(int shooter_time)//蓝方LongGoal筛球发射，括号内为发射时间，单位毫秒
{
  Shooter_Optical.setLight(ledState::on);
  int shooter_time_now = 0;
  LMove.spin(fwd,-10,pct);
  RMove.spin(fwd,-10,pct);
  while(shooter_time_now <= shooter_time/20)
  {
    Intake.spin(fwd,100,pct);
    DownRoller.spin(fwd,100,pct);
    if(!(Shooter_Optical.hue() > 350 or Shooter_Optical.hue() < 30))
    {
      UpRoller.spin(fwd,60,pct);
    }
    if(!(Shooter_Optical.hue() > 350 or Shooter_Optical.hue() < 30))
    {
      Shooter.spin(fwd,100,pct);
    }
    else if(Shooter_Optical.hue() > 350 or Shooter_Optical.hue() < 30)
    {
      Shooter.spin(fwd,-100,pct);
    }
    shooter_time_now += 1;//计数器
    wait(20, msec);//小延迟防止CPU过载
  }
  Intake.stop();
  DownRoller.stop();
  UpRoller.stop();
  Shooter.stop();
  LMove.stop();
  RMove.stop();
  Brain.Screen.print("Backward for 1s...");

  LMove.spin(fwd,30,pct);
  RMove.spin(fwd,30,pct);
  wait(600,msec);
  GPS_TurnToHeading(180);
  LMove.spin(fwd,-50,pct);
  RMove.spin(fwd,-50,pct);
  wait(500,msec);
  GPS_TurnToHeading(270);
  // 停止驱动电机（刹车模式）
  LMove.stop(brake);
  RMove.stop(brake);
  Brain.Screen.clearLine(9);
  Brain.Screen.print("Backward done!");
}

/*---------------------------------------------------------------------------*/
/*                          可复用的跟踪启动函数                             */
/*---------------------------------------------------------------------------*/
const int MAX_GLOBAL_CYCLES = 3; 
void runTrackingAndGPSCycle() { // 函数名可调整，更清晰
  int global_cycle_count = 0; // 全局循环计数器（跟踪→GPS为1个全局循环）

  // 检查串口是否打开
  if(serial_fp == NULL) {
    Brain.Screen.print("Serial port not open!");
    return;
  }

  // 循环执行：直到达到最大次数 或 手动停止
  while (global_cycle_count < MAX_GLOBAL_CYCLES || MAX_GLOBAL_CYCLES == 0) {
    // 每次循环前重置跟踪状态变量
    track_cycle = 0;
    tracking_complete = false;
    is_tracking_stuck = false;
    last_active_time = Brain.timer(msec);
    last_gps_x = GPS.xPosition(mm);
    last_gps_y = GPS.yPosition(mm);

    global_cycle_count++;
    Brain.Screen.clearLine(5);
    Brain.Screen.print("Global Cycle: %d/%d | Restart tracking...", 
      global_cycle_count, MAX_GLOBAL_CYCLES == 0 ? 999 : MAX_GLOBAL_CYCLES);

    // 第一阶段：自旋搜索初始目标
    bool found_blue = false;
    Brain.Screen.clearLine(6);
    Brain.Screen.print("Spinning to find initial blue...");
    
    Drivetrain.turn(right, turn_speed * 1.5, rpm);
    
    while(!found_blue && !checkIfStuck() && serial_fp != NULL) {
      char buffer[128] = {0};
      if(fgets(buffer, sizeof(buffer), serial_fp) != NULL) {
        size_t len = std::strlen(buffer);
        if(len>0 && (buffer[len-1]=='\n'||buffer[len-1]=='\r')) {
          buffer[len-1] = '\0';
        }
        std::string cmd_str(buffer);
        std::vector<std::string> parts = split(cmd_str, ',');
        
        if(parts.size() > 0 && parts[0] == desired_detection) {
          found_blue = true;
          Drivetrain.stop();
          Brain.Screen.clearLine(6);
          Brain.Screen.print("Found initial blue! Starting tracking...");
          last_active_time = Brain.timer(msec);
          last_gps_x = GPS.xPosition(mm);
          last_gps_y = GPS.yPosition(mm);
        }
      }
      vex::this_thread::sleep_for(2);
    }

    // 第二阶段：三次目标跟踪循环
    while(track_cycle < MAX_TRACK_CYCLE && !checkIfStuck() && serial_fp != NULL) {
      char buffer[128] = {0};
      if(fgets(buffer, sizeof(buffer), serial_fp) != NULL) {
        size_t len = std::strlen(buffer);
        if(len>0 && (buffer[len-1]=='\n'||buffer[len-1]=='\r')) {
          buffer[len-1] = '\0';
        }
        std::string cmd_str(buffer);
        handleCommand(cmd_str);
      }
      if (checkIfStuck()) {
        Brain.Screen.clearLine(5);
        Brain.Screen.print("Cycle %d: Stuck detected! Skip tracking", global_cycle_count);
        break;
      }
      vex::this_thread::sleep_for(2);
    }

    // 若跟踪未卡死，执行GPS流程；否则跳过GPS，直接进入下一轮
    if (!is_tracking_stuck) {
      Brain.Screen.clearLine(5);
      Brain.Screen.print("Cycle %d: All tracks done! Start GPS...", global_cycle_count);
      BlueLeftShoot(); // 执行GPS定位与发射（已移除位置重置）
    } else {
      // 卡死时不再重置GPS位置，仅重置卡死标志
      Brain.Screen.clearLine(5);
      Brain.Screen.print("Cycle %d: Stuck! Skip position reset...", global_cycle_count);
      is_tracking_stuck = false; // 重置卡死标志，允许下一轮
    }

    // 可选：每次全局循环后延迟1秒，避免连续触发
    wait(1, sec);
  }

  // 所有全局循环结束后，停止所有动作
  Brain.Screen.clearLine(5);
  Brain.Screen.print("All global cycles completed! Stop.");
  Drivetrain.stop(brake);
  Intake.stop(coast);
  DownRoller.stop(coast);
  UpRoller.stop(coast);
  Shooter.stop(coast);
}

/*---------------------------------------------------------------------------*/
/*                          Auto_Isolation Task                              */
/*---------------------------------------------------------------------------*/
void auto_Isolation(void) {
  // ..........................................................................
  // 预留隔离阶段代码位置，可在此添加自定义逻辑
  // 示例：仅打印阶段信息
  Brain.Screen.print("Isolation Phase: Reserved for custom code");
  GPS_YFUMove(-1320);
  Loader.set(1);
  GPS_TurnToRotation(90);
  wait(300,msec);
  Intake.spin(fwd,100,pct);
  DownRoller.spin(fwd,100,pct);
  // UpRoller.spin(fwd,30,pct);
  LMove.spin(fwd,40,pct);
  RMove.spin(fwd,40,pct);
  wait(0.3,sec);
  LMove.spin(fwd,20,pct);
  RMove.spin(fwd,20,pct);
  wait(2,sec);
  LMove.spin(fwd,-30,pct);
  RMove.spin(fwd,-30,pct);
  wait(1.5,sec);
  Loader.set(0);
  UpRoller.spin(fwd,30,pct);
  BlueLG_Shoot(3000);


  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                        Auto_Interaction Task                              */
/*---------------------------------------------------------------------------*/
void auto_Interaction(void) {
  // 交互阶段执行完整的跟踪+GPS+发射逻辑
  Brain.Screen.print("Interaction Phase: Start Tracking & GPS");
  pre_autonomous(); // 确保GPS校准和串口初始化
  runTrackingAndGPSCycle(); // 执行核心跟踪逻辑
  BlueLG_Shoot(5000); // 执行LongGoal发射逻辑（可选，可根据需求调整）
}

/*---------------------------------------------------------------------------*/
/*                          AutonomousMain Task                              */
/*---------------------------------------------------------------------------*/
bool firstAutoFlag = true;

void autonomousMain(void) {
  // 第一次进入执行隔离阶段，第二次进入执行交互阶段
  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

/*----------------------------------------------------------------------------*/
int main() {
    // 替换vexcodeInit()的手动初始化逻辑
    Brain.Screen.clearScreen();
    GPS.calibrate();
    while(GPS.isCalibrating()) wait(50, msec);
    serial_fp = fopen("/dev/serial1", "r");

    // 设置自主阶段回调函数
    Competition.autonomous(autonomousMain);

    // 主循环保持运行
    while (true) {
      wait(100, msec);
    }
}