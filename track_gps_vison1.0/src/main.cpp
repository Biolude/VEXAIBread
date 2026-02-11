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


// 跟踪配置参数
std::string desired_detection = "blue";
double desired_x = 310;         // 目标x坐标（中心位置）
double x_tolerance = 60;        // x方向允许误差范围（像素）
double target_y = 380;          // 目标y坐标阈值
double forward_speed = 250;     // 前进速度（rpm）
double turn_speed = 40;         // 转向速度（rpm）
double Intake_speed = 400;      // 3号电机转速（rpm）
double DownRoller_speed = 400;  // 5号电机转速（rpm）
bool tracking_complete = false; // 单次跟踪完成标志
// 新增：跟踪次数计数器与最大次数（控制三次循环）
int track_cycle = 0;            // 当前完成的跟踪循环次数
const int MAX_TRACK_CYCLE = 3;  // 总跟踪循环次数（三次）

// ===================== 新增：卡死检测相关变量 =====================
bool is_tracking_stuck = false; // 卡死标志
uint32_t last_active_time = 0;  // 最后一次“有效活动”的时间戳（ms）
const uint32_t STUCK_TIMEOUT = 5000; // 卡死判定阈值（5秒，可根据需求调整）
double last_gps_x = 0.0;        // 上一次GPS X坐标
double last_gps_y = 0.0;        // 上一次GPS Y坐标
const double POS_CHANGE_THRESHOLD = 100.0; // 位置变化阈值（100mm，移动超过此值判定为有效活动）
// =================================================================

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

// ===================== 新增：卡死检测函数 =====================
bool checkIfStuck() {
  // 如果已判定卡死，直接返回true
  if (is_tracking_stuck) return true;

  // 获取当前时间和GPS位置
  uint32_t current_time = Brain.timer(msec);
  double current_gps_x = GPS.xPosition(mm);
  double current_gps_y = GPS.yPosition(mm);

  // 计算位置变化量（欧几里得距离）
  double pos_change = sqrt(pow(current_gps_x - last_gps_x, 2) + pow(current_gps_y - last_gps_y, 2));

  // 判定1：位置变化超过阈值 → 有效活动，重置计时器和位置
  if (pos_change > POS_CHANGE_THRESHOLD) {
    last_active_time = current_time;
    last_gps_x = current_gps_x;
    last_gps_y = current_gps_y;
    Brain.Screen.clearLine(7);
    Brain.Screen.print("Active! Pos change: %.1fmm", pos_change);
    return false;
  }

  // 判定2：超过超时时间且位置无有效变化 → 判定为卡死
  if (current_time - last_active_time > STUCK_TIMEOUT) {
    is_tracking_stuck = true;
    Brain.Screen.clearLine(7);
    Brain.Screen.print("STUCK DETECTED! Timeout: %dms", STUCK_TIMEOUT);
    // 卡死时停止所有电机
    Drivetrain.stop();
    return true;
  }

  // 未卡死
  return false;
}
// =================================================================

// 命令处理函数（跟踪逻辑）- 核心修改处
void handleCommand(const std::string &cmd) 
{
  // 若三次跟踪已完成 或 已卡死，不再处理命令
  if (track_cycle >= MAX_TRACK_CYCLE || is_tracking_stuck) return;
  
  std::vector<std::string> parts = split(cmd, ',');
  std::string detection_type = parts[0];
  Brain.Screen.clearLine(1);
  Brain.Screen.print("Recv cmd: %s | Cycle: %d/%d", cmd.c_str(), track_cycle + 1, MAX_TRACK_CYCLE);

  // 如果检测到目标且当前处于等待状态，重置跟踪标志
  if (detection_type == desired_detection && tracking_complete) {
    tracking_complete = false;  // 重置单次跟踪标志
    Brain.Screen.clearLine(4);
    Brain.Screen.print("Found new target! Restart tracking...");
  }

  // 若单次跟踪未完成，继续处理
  if (tracking_complete) return;
  
  // 如果未检测到任何目标，停止不动
  if (detection_type == "none") 
  {
    // Drivetrain.stop();
    LMove.spin(reverse,turn_speed *1.5,rpm);
    RMove.spin(reverse,turn_speed *0.5,rpm);
    return;
  }

  // 只处理期望的目标类型（blue）
  if(detection_type == desired_detection && parts.size() >= 3) 
  {
    Intake.spin(forward, Intake_speed, rpm);
    DownRoller.spin(forward, DownRoller_speed, rpm);
    double current_x = std::atof(parts[1].c_str());
    double current_y = std::atof(parts[2].c_str());
    double x_error = desired_x - current_x;

    // 显示当前坐标信息与跟踪进度
    Brain.Screen.clearLine(2);
    Brain.Screen.print("X: %.1f, Y: %.1f | Target: %s", current_x, current_y, desired_detection.c_str());

    // 当y大于0且小于设定坐标时，正常前进
    if (current_y > 0 && current_y < target_y) 
    {
      // 判断x方向是否在允许误差范围内
      if (std::fabs(x_error) <= x_tolerance) 
      {
        // x方向对齐，向前移动
        Drivetrain.drive(forward, forward_speed, rpm);
      } 
      else 
      {
        // x方向未对齐，转向调整（左/右）
        if (x_error > 0)
        {
          // Drivetrain.turn(left, turn_speed, rpm);
          LMove.stop();
          RMove.spin(fwd,turn_speed,rpm);
        }
          
        else
        {
          // Drivetrain.turn(right, turn_speed, rpm);
          RMove.stop();
          LMove.spin(fwd,turn_speed,rpm);
        }
      }
    } 
    else if (current_y >= target_y)  // 当y大于等于430时
    {
      // 前进1秒后停下
      Drivetrain.drive(forward, forward_speed, rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Y >= 430, Moving forward for 1s...");
  
      wait(1, sec);
      
      // 停止移动与执行电机
      Drivetrain.stop();
      // Intake.stop();
      // DownRoller.stop();
      
      // 等待2秒
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Waiting for 2s...");
      wait(2, sec);
      tracking_complete = true;
      track_cycle++;
      // 标记单次跟踪完成，跟踪循环次数+1
      // 新增：完成单次跟踪时重置卡死检测状态
      last_active_time = Brain.timer(msec);
      last_gps_x = GPS.xPosition(mm);
      last_gps_y = GPS.yPosition(mm);

      Brain.Screen.clearLine(3);
      Brain.Screen.print("Single track done! Total: %d/%d", track_cycle, MAX_TRACK_CYCLE);

      // 三次未完成则自旋寻找下一个目标
      if (track_cycle < MAX_TRACK_CYCLE) 
      {
        Brain.Screen.clearLine(4);
        Brain.Screen.print("Spinning to find next %s...", desired_detection.c_str());
        // 原地右自旋2秒
        // Drivetrain.turn(right, turn_speed * 0.8, rpm);
        LMove.spin(reverse,turn_speed *1.5,rpm);
        RMove.spin(reverse,turn_speed *0.5,rpm);
        wait(2, sec);
        // Drivetrain.stop();
        
        Brain.Screen.clearLine(4);
        Brain.Screen.print("Waiting for next target...");
      }
    } 
    else  // 当y小于等于0时，寻找目标
    {
      // Drivetrain.stop();
      LMove.spin(reverse,turn_speed *1.5,rpm);
      RMove.spin(reverse,turn_speed *0.5,rpm);
      Brain.Screen.clearLine(3);
      Brain.Screen.print("Y <= 0, waiting for valid target...");
    }
  }
  else
  {
    // 非目标类型，继续寻找
    // Drivetrain.stop();
    LMove.spin(reverse,turn_speed *1.5,rpm);
    RMove.spin(reverse,turn_speed *0.5,rpm);
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

  // ===================== 新增：初始化卡死检测变量 =====================
  last_active_time = Brain.timer(msec);
  last_gps_x = GPS.xPosition(mm);
  last_gps_y = GPS.yPosition(mm);
  // =================================================================
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
      }//转向分为两段，距离目标较远使用高速快速转向，距离目标角度较近低速转向确保稳定
    }while(GPS.heading(deg) > heading_deg + 12 or GPS.heading(deg) < heading_deg - 12);//角度误差范围±12°
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);//停止后短暂等待，确保稳定
}

void GPS_XMove(int Xdis)//GPS沿X轴运动(对应角度90/270°),变量为目标X坐标
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
    }while(GPS.xPosition(mm)  > Xmove_distance + 100 or GPS.xPosition(mm) < Xmove_distance - 100);//X轴位置误差范围±100mm
    LMove.stop(hold);
    RMove.stop(hold);
    wait(0.5,msec);
}

void GPS_YMove(int Ydis)//GPS沿Y轴运动(对应角度0/180°),变量为目标Y坐标
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
    }while(GPS.yPosition(mm)  > Ymove_distance + 100 or GPS.yPosition(mm) < Ymove_distance - 100);//Y轴位置误差范围±100mm
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
  pre_autonomous(); // 初始化：陀螺仪+GPS校准
  FILE *fp = fopen("/dev/serial1", "r");
  if(!fp) {
    Brain.Screen.print("Failed to open serial1!");
    return 1;
  }
  Brain.Screen.print("Start blue tracking (3 cycles)...");

  // 新增：自旋搜索目标阶段
  bool found_blue = false;
  Brain.Screen.clearLine(6);
  Brain.Screen.print("Spinning to find initial blue...");
  
  // 开始原地自旋搜索
  Drivetrain.turn(right, turn_speed * 1.5, rpm); // 慢速自旋
  
  while(!found_blue && !checkIfStuck()) { // 新增：自旋阶段也检测卡死
    char buffer[128] = {0};
    if(fgets(buffer, sizeof(buffer), fp) != NULL) {
      // 去除换行符
      size_t len = std::strlen(buffer);
      if(len>0 && (buffer[len-1]=='\n'||buffer[len-1]=='\r')) {
        buffer[len-1] = '\0';
      }
      std::string cmd_str(buffer);
      std::vector<std::string> parts = split(cmd_str, ',');
      
      // 检测到blue目标则停止自旋
      if(parts.size() > 0 && parts[0] == desired_detection) {
        found_blue = true;
        Drivetrain.stop();
        Brain.Screen.clearLine(6);
        Brain.Screen.print("Found initial blue! Starting tracking...");
        // 重置卡死检测状态
        last_active_time = Brain.timer(msec);
        last_gps_x = GPS.xPosition(mm);
        last_gps_y = GPS.yPosition(mm);
      }
    }
    vex::this_thread::sleep_for(2);
  }

  // 第二阶段：三次目标跟踪循环（新增卡死检测）
  while(track_cycle < MAX_TRACK_CYCLE && !checkIfStuck()) {
    char buffer[128] = {0};
    if(fgets(buffer, sizeof(buffer), fp) != NULL) {
      // 去除换行符
      size_t len = std::strlen(buffer);
      if(len>0 && (buffer[len-1]=='\n'||buffer[len-1]=='\r')) {
        buffer[len-1] = '\0';
      }
      std::string cmd_str(buffer);
      handleCommand(cmd_str); // 处理串口接收的目标检测命令
    }
    // 每次循环都检测是否卡死
    if (checkIfStuck()) {
      Brain.Screen.clearLine(5);
      Brain.Screen.print("Force exit tracking! Stuck detected");
      break; // 跳出跟踪循环
    }
    vex::this_thread::sleep_for(2);
  }

  // 三次跟踪完成 或 卡死 → 关闭串口并执行GPS
  fclose(fp);
  Brain.Screen.clearLine(5);
  if (is_tracking_stuck) {
    Brain.Screen.print("Stuck! Skip tracking, run GPS...");
  } else {
    Brain.Screen.print("All 3 tracks done! Start GPS...");
  }
  
  // 执行GPS定位与发射程序
  RedLeftShoot();

  // 主循环保持运行
  while (true) {
    wait(100, msec);
  }
}