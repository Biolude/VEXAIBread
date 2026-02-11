/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Combined Tracking and GPS Program                         */
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
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of competition
competition Competition;
brain  Brain;
bool RemoteControlCodeEnabled = true;

// 驱动电机配置
motor LMoveMotorA = motor(PORT14, ratio6_1, false);
motor LMoveMotorB = motor(PORT12, ratio6_1, false);
motor_group LMove = motor_group(LMoveMotorA, LMoveMotorB);
motor RMoveMotorA = motor(PORT11, ratio6_1, true);
motor RMoveMotorB = motor(PORT13, ratio6_1, true);
motor_group RMove = motor_group(RMoveMotorA, RMoveMotorB);
drivetrain Drivetrain = drivetrain(LMove, RMove, 319.024, 284.48, 196.85, mm, 1);

// 其他电机和传感器配置
controller Controller1 = controller(primary);
motor Intake = motor(PORT5, ratio6_1, true);
motor Shooter = motor(PORT2, ratio6_1, false);
digital_out Loader = digital_out(Brain.ThreeWirePort.H);
inertial Gyro = inertial(PORT6);
gps GPS = gps(PORT7, 88.9, -177.8, mm, 180);

// 前吸和后吸电机


// 跟踪配置参数
std::string desired_detection = "blue";
double desired_x = 317;         // 目标x坐标（中心位置）
double x_tolerance = 50;        // x方向允许误差范围（像素）
double min_y = 80;             // 最小y坐标（过近阈值）
double max_y = 400;             // 最大y坐标（过远阈值）
double forward_speed = 100;     // 前进速度（rpm）
double turn_speed = 10;         // 转向速度（rpm）
double Intake_speed = 400;      // 3号电机转速（rpm）
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

// 命令处理函数（跟踪逻辑）
/*
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
    return;
  }

  // 只处理期望的目标类型
  if(detection_type == desired_detection && parts.size() >= 3) 
  {
    Brain.Screen.print("This part is triggering!");
    Intake.spin(forward, Intake_speed, rpm);
    //DownRoller.spin(forward, DownRoller_speed, rpm);
    double current_x = std::atof(parts[1].c_str());
    double current_y = std::atof(parts[2].c_str());
    double x_error = desired_x - current_x;

    // 显示当前坐标信息
    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f", current_x, current_y);

    // 判断是否在有效y范围（目标距离合适）
    //The inputted data has to be within the camera frame boundries 
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
        // x方向未对齐，转向调整
        if (x_error < 0)
          Drivetrain.turn(left, turn_speed, rpm);
        else
          Drivetrain.turn(right, turn_speed, rpm);
      }
    } 
    else if (current_y < min_y) 
    {
      // 目标过近，停止移动并标记跟踪完成
      Drivetrain.stop();
      Intake.stop();
      //DownRoller.stop();
      tracking_complete = true;
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Tracking complete!");
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
    Drivetrain.stop();
  }
}
  */

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
/*                          GPS相关函数                                      */
/*---------------------------------------------------------------------------*/
void pre_autonomous(void) {
  Gyro.calibrate(); // 陀螺仪校准
  while(Gyro.isCalibrating()) wait(50, msec);
  
  GPS.calibrate();
  while(GPS.isCalibrating()) wait(50, msec);
  GPS.setRotation(0,deg);
  
}

void GPS_TurnToHeading(float Heading)//GPS转向函数,变量为目标方向
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
      if(GPS.heading(deg) < heading_deg - 12 and GPS.heading(deg) > heading_deg - 30)
      {
        LMove.spin(fwd,5,pct);
        RMove.spin(fwd,-5,pct);
      }
      else if(GPS.heading(deg) > heading_deg + 12 and GPS.heading(deg) < heading_deg + 30)
      {
        LMove.spin(fwd,-5,pct);
        RMove.spin(fwd,5,pct);
      }//转向分为两段，距离目标较远使用高速快速转向，距离目标角度较近低速转向确保稳定，速度可以根据机器进行调整
    }while(GPS.heading(deg) > heading_deg + 12 or GPS.heading(deg) < heading_deg - 12);//判断是否到达目标角度，目标角度范围可根据需要调整
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);//停止移动并等待
}

void GPS_XMove(int Xdis)//GPS以场地X轴运动(度数90/270),变量为目标X距离
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
    }while(GPS.xPosition(mm)  > Xmove_distance + 100 or GPS.xPosition(mm) < Xmove_distance - 100);//判断是否到达目标X位置，目标X位置范围可根据需要调整
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);//停止移动并等待
}

void GPS_YMove(int Ydis)//GPS以场地Y轴运动(度数0/180),变量为目标Y距离
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
    }while(GPS.yPosition(mm)  > Ymove_distance + 100 or GPS.yPosition(mm) < Ymove_distance - 100);//判断是否到达目标X位置，目标X位置范围可根据需要调整
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);//停止移动并等待
}

void RedLeftShoot()
{
  /*
    while(true)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Heading: %.2f", GPS.heading(deg));
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Rotation: %.2f", GPS.rotation(deg));
    Brain.Screen.setCursor(3,1);
    Brain.Screen.print("X: %.2f mm", GPS.xPosition(mm));
    Brain.Screen.setCursor(4,1);
    Brain.Screen.print("Y: %.2f mm", GPS.yPosition(mm));

    wait(1, sec);
  }
    */
    
    GPS_TurnToHeading(90);//GPS传感器对准红站位
    GPS_XMove(-340);//后退到红方区域
    GPS_TurnToHeading(180);//转向红左
    GPS_YMove(420);//对齐左侧long goal
    GPS_TurnToHeading(316);//前吸后打机型对准左侧long goal（前吸前打机型应改为90度）

    LMove.spin(fwd,-50,pct);
    RMove.spin(fwd,-50,pct);
    wait(1,sec);//顶框，需要机器具备long goal限位结构
    Intake.spin(fwd,100,pct);
    //DownRoller.spin(fwd,100,pct);
    //UpRoller.spin(fwd,100,pct);
    //Shooter.spin(fwd,100,pct);
    LMove.spin(fwd,-10,pct);
    RMove.spin(fwd,-10,pct);
    wait(5, sec);//顶住long goal发射
    Intake.stop(coast);
    //DownRoller.stop(coast);
    //UpRoller.stop(coast);
    //Shooter.stop(coast);
    LMove.stop(brake);
    RMove.stop(brake);
    
    
}

/*---------------------------------------------------------------------------*/
/*                              主程序流程                                   */
/*---------------------------------------------------------------------------*/

int main() {
  pre_autonomous(); 
  
  
    
  FILE *fp = fopen("/dev/serial1", "r");
  if(!fp) {
    Brain.Screen.print("Failed to open /dev/serial1");
    return 1;
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
  
  // 原地等待2秒
  Brain.Screen.clearLine(4);
  Brain.Screen.print("Waiting 2 seconds...");
  wait(2, sec);

  // 第二阶段：执行GPS定位程序
  Brain.Screen.clearLine(4);
  Brain.Screen.print("Starting GPS program...");
  RedLeftShoot();

  // 主循环保持程序运行
  while (true) {
    wait(100, msec);
  }
}