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
inertial Gyro = inertial(PORT13);
gps GPS = gps(PORT10, -87.50, -100.00, mm, 180);

// 前吸和后吸电机
motor motor3(PORT3, ratio6_1, false);
motor motor5(PORT5, ratio6_1, true);

// 跟踪配置参数
std::string desired_detection = "blue";
double desired_x = 300;         // 目标x坐标（中心位置）
double x_tolerance = 50;        // x方向允许误差范围（像素）
double forward_speed = 200;     // 前进速度（rpm）
double Intake_speed = 400;      // 3号电机转速（rpm）
double DownRoller_speed = 400;      // 5号电机转速（rpm）
double turn_speed = 100;        // 转向速度（rpm）
double search_spin_speed = 50;  // 搜索自旋速度（rpm）
double lost_forward_time = 2.0; // 目标丢失后前进时间（秒）

// 跟踪状态变量
timer approach_timer;
bool is_approaching = false;
bool current_tracking_complete = false;  // 当前跟踪完成标志
int tracking_count = 0;                  // 跟踪次数计数
const int total_tracking = 3;            // 总跟踪次数
bool target_lost = false;                // 目标丢失标志
timer lost_timer;                        // 目标丢失计时器

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

// 重置单次跟踪状态
void resetSingleTracking() {
  current_tracking_complete = false;
  is_approaching = false;
  target_lost = false;
  approach_timer.reset();
  lost_timer.reset();
}

// 原地旋转准备下一次跟踪
void rotate180ForNextTracking() {
  Brain.Screen.clearLine(3);
  Brain.Screen.print("Rotating 180 degrees...");
  
  // 记录开始时间
  timer rotate_timer;
  rotate_timer.reset();
  
  // 旋转（根据实际情况调整时间）
  LMove.spin(reverse, search_spin_speed * 1.5, rpm);
  RMove.spin(reverse, search_spin_speed * 0.5, rpm);
  
  // 旋转的时间（根据实际测试调整）
  wait(2, sec);
  
  // 停止旋转
  LMove.stop();
  RMove.stop();
  wait(0.5, sec);
}

// 命令处理函数（跟踪逻辑）
void handleCommand(const std::string &cmd) 
{
  if (current_tracking_complete) return; // 当前跟踪完成后不再处理命令
  
  std::vector<std::string> parts = split(cmd, ',');
  std::string detection_type = parts[0];
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Recv cmd: %s", cmd.c_str());

  // 确保至少有3个部分（类型、x、y）
  if (parts.size() < 3) {
    return; // 无效命令格式，忽略
  }

  // 解析坐标值
  double current_x = std::atof(parts[1].c_str());
  double current_y = std::atof(parts[2].c_str());

  // 处理目标丢失情况（Python发送-1,-1表示未检测到目标）
  if (current_x == -1 && current_y == -1) {
    // 已发现过目标后丢失
    if (target_lost) {
      // 目标丢失后向前行驶指定时间
      Drivetrain.drive(forward, forward_speed, rpm);
      if (lost_timer.time(sec) >= lost_forward_time) {
        Drivetrain.stop();
        current_tracking_complete = true;
        Brain.Screen.clearLine(3);
        Brain.Screen.print("Tracking %d complete (lost)", tracking_count);
      }
    } 
    // 未发现目标且未开始跟踪时，原地自旋搜索
    else {
      Drivetrain.stop();
      // 原地自旋搜索目标
      LMove.spin(reverse, search_spin_speed*1.5, rpm);
      RMove.spin(reverse, search_spin_speed*0.5, rpm);
    }
    return;
  }

  // 发现目标后停止搜索自旋
  LMove.stop();
  RMove.stop();

  // 只处理期望的目标类型
  if(detection_type == desired_detection) 
  {
    target_lost = true;  // 标记已发现目标
    Intake.spin(forward, Intake_speed, rpm);
    DownRoller.spin(forward, DownRoller_speed, rpm);

    // 显示当前坐标信息
    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f", current_x, current_y);

    // 检查X方向是否在允许范围内
    bool x_aligned = std::fabs(desired_x - current_x) <= x_tolerance;

    if (!x_aligned) 
    {
      // x方向未对齐，进行自旋调整
      double x_deviation = current_x - desired_x;
      // 目标在右侧（x_deviation>0）-> 向右转: 左轮前进，右轮后退
      if (x_deviation > 0) {
        LMove.stop();
        RMove.spin(reverse, turn_speed, rpm);
      } 
      // 目标在左侧（x_deviation<0）-> 向左转: 左轮后退，右轮前进
      else {
        LMove.spin(reverse, turn_speed, rpm);
        RMove.stop();
      }
      is_approaching = false;
    }
    // X方向对齐后，处理Y方向移动
    else 
    {
      // 停止转向电机（确保不再进行左右调整）
      LMove.stop();
      RMove.stop();
      
      // X已对齐，检查Y方向
      if (current_y < 460) 
      {
        // Y小于460，向前直行
        Drivetrain.drive(forward, forward_speed, rpm);
        is_approaching = false;
      } 
      else if (current_y > 460) 
      {
        // Y大于460，向前行进2秒后停止
        if (!is_approaching) 
        {
          is_approaching = true;
          approach_timer.reset();
          Drivetrain.drive(forward, forward_speed, rpm);
        } 
        else if (approach_timer.time(sec) < 2.0)
        {
          Drivetrain.drive(forward, forward_speed, rpm);
        } 
        else 
        {
          // 2秒后停止所有动作，标记当前跟踪完成
          Drivetrain.stop();
          is_approaching = false;
          current_tracking_complete = true;
          Brain.Screen.clearLine(3);
          Brain.Screen.print("Tracking %d complete", tracking_count);
        }
      }
    }
  }
  else
  {
    // 非目标类型，停止不动
    Drivetrain.stop();
    is_approaching = false;
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
}

void GPS_TurnToHeading(float Heading)//GPS转向函数
{
  heading_deg = Heading;
  do{
      if(GPS.heading(deg) < heading_deg - 30)
      {
        LMove.spin(fwd,30,pct);
        RMove.spin(fwd,-30,pct);
      }
      else if(GPS.heading(deg) > heading_deg + 30)
      {
        LMove.spin(fwd,-30,pct);
        RMove.spin(fwd,30,pct);
      }
      if(GPS.heading(deg) < heading_deg - 12 and GPS.heading(deg) > heading_deg - 30)
      {
        LMove.spin(fwd,10,pct);
        RMove.spin(fwd,-10,pct);
      }
      else if(GPS.heading(deg) > heading_deg + 12 and GPS.heading(deg) < heading_deg + 30)
      {
        LMove.spin(fwd,-10,pct);
        RMove.spin(fwd,10,pct);
      }
    }while(GPS.heading(deg) > heading_deg + 12 or GPS.heading(deg) < heading_deg - 12);
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);
}

void GPS_XMove(int Xdis)//GPS X轴运动(度数90/270)
{
  Xmove_distance = Xdis;
  do{
      if(GPS.xPosition(mm)  < Xmove_distance - 100)
      {
        LMove.spin(fwd,40,pct);
        RMove.spin(fwd,40,pct);
      }
      else if(GPS.xPosition(mm)  > Xmove_distance + 100)
      {
        LMove.spin(fwd,-40,pct);
        RMove.spin(fwd,-40,pct);
      }
    }while(GPS.xPosition(mm)  > Xmove_distance + 100 or GPS.xPosition(mm) < Xmove_distance - 100);
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);
}

void GPS_YMove(int Ydis)//GPS Y轴运动(度数0/180)
{
  Ymove_distance = Ydis;
  do{
      if(GPS.yPosition(mm)  < Ymove_distance - 100)
      {
        LMove.spin(fwd,-40,pct);
        RMove.spin(fwd,-40,pct);
      }
      else if(GPS.yPosition(mm)  > Ymove_distance + 100)
      {
        LMove.spin(fwd,40,pct);
        RMove.spin(fwd,40,pct);
      }
    }while(GPS.yPosition(mm)  > Ymove_distance + 100 or GPS.yPosition(mm) < Ymove_distance - 100);
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);
}

void RedLeftShoot()
{
    if(GPS.xPosition(mm) > 700)
    {
      GPS_TurnToHeading(180);
      GPS_YMove(600);
    }
    else if(GPS.xPosition(mm) > -700 and GPS.xPosition(mm) < 700)
    {
      if(GPS.xPosition(mm) > 0 and GPS.xPosition(mm) < 1000)
      {
        GPS_TurnToHeading(180);
        GPS_YMove(600);
      }
      else if(GPS.xPosition(mm) > -1000 and GPS.xPosition(mm) < 0)
      {
        GPS_TurnToHeading(180);
        GPS_YMove(-600);
      }
      
    }
    GPS_TurnToHeading(90);//GPS对准红方站位
    GPS_XMove(-1200);//后退到红方区域（X轴目标坐标-1200mm）
    GPS_TurnToHeading(180);//转向红方左侧
    GPS_YMove(1300);//沿Y轴移动到左侧long goal对齐位置
    GPS_TurnToHeading(270);//前吸后打机型：对准左侧long goal（前吸前打机型需改为90°）
    LMove.spin(fwd,-50,pct);
    RMove.spin(fwd,-50,pct);
    wait(1,sec);//顶框（需机器具备long goal限位结构）
    Intake.spin(fwd,100,pct);
    DownRoller.spin(fwd,100,pct);
    UpRoller.spin(fwd,100,pct);
    Shooter.spin(fwd,100,pct);
    LMove.spin(fwd,-10,pct);
    RMove.spin(fwd,-10,pct);
    wait(5, sec);//顶住long goal持续发射
    // 停止所有电机（coast：自由停止；brake：制动停止）
    Intake.stop(coast);
    DownRoller.stop(coast);
    UpRoller.stop(coast);
    Shooter.stop(coast);
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

  // 执行三次跟踪
  while(tracking_count < total_tracking) 
  {
    tracking_count++;
    resetSingleTracking();  // 重置单次跟踪状态
    Brain.Screen.clearLine(4);
    Brain.Screen.print("Tracking %d started", tracking_count);
    
    // 单次跟踪循环
    while(!current_tracking_complete) 
    {
      Brain.Screen.clearLine(5);  // 使用第5行显示跟踪次数
      Brain.Screen.print("当前跟踪: %d/%d", tracking_count, total_tracking);
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
    
    // 单次跟踪完成后等待1秒
    Brain.Screen.clearLine(4);
    Brain.Screen.print("Tracking %d finished. Waiting 1s...", tracking_count);
    wait(1, sec);
    
    // 如果不是最后一次跟踪，则原地旋转180度准备下一次
    if(tracking_count < total_tracking) {
      rotate180ForNextTracking();
    }
  }

  // 三次跟踪完成后关闭串口
  fclose(fp);

  // 等待2秒后进入GPS程序
  Brain.Screen.clearLine(4);
  Brain.Screen.print("All tracking complete. Waiting 2s...");
  wait(2, sec);

  // 执行GPS定位程序
  Brain.Screen.clearLine(4);
  Brain.Screen.print("Starting GPS program...");
  RedLeftShoot();

  // 主循环保持程序运行
  while (true) {
    wait(100, msec);
  }
}