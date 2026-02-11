# GPS 定位示例（基于 VEX V5 C++ API）

说明：这是一个使用 VEX V5 `gps` 传感器的示例程序，程序在 Brain 屏幕上显示 X/Y/Heading/Quality，并包含一个简单的基于 X 坐标的移动示例。

主要文件：
- `src/main.cpp`：GPS 示例程序。

使用说明（推荐使用 VEXcode V5）：

1. 在 VEXcode V5 中打开或创建项目，并将代码替换为本仓库的 `src/main.cpp`（或直接在 VEXcode 中粘贴）。
2. 在 Devices（设备）窗口中添加 GPS 设备，并设置端口为 `PORT20`（或根据你实际连接的端口修改 `src/main.cpp` 中的构造）。
   - 如果 GPS 不是放在机器人参考点处，请在设备配置中设置 X/Y 偏移和角度偏移，或在代码中使用 `gpsSensor.setOrigin(...)` / `gpsSensor.setLocation(...)`。
3. 编译并下载到 V5 Brain，运行程序。Brain 屏幕将显示当前 X、Y、Heading 和 Quality。代码示例在 Quality 足够高时会让机器人前进到 X >= 1000 mm。

命令行构建（可选）：

如果你使用命令行或已有的 makefile，确保使用 VEX 官方工具链（或 VEXcode CLI）。通常工作流程类似：

```powershell
# 在项目根目录
make
# 或者使用 VEXcode 的导出项目并用 VEX 工具链编译
```

注意事项：
- 程序示例中 `drivetrain` 的参数（轮间距、轮直径）需要根据你的底盘进行调整，以保证运动精度。
- GPS 信号质量（`gpsSensor.quality()`）在开始运动前最好大于 ~80；较低时坐标可能不可靠。
- 如果机器人在靠近场地边缘启动，建议使用 `gpsSensor.setLocation(...)` 提供已知起始坐标以提高准确性。

如果你希望我：
- 为你的底盘精确计算 `drivetrain` 参数（需要轮径与轴距），或
- 添加更复杂的导航（路径规划、PID 控制等），
请告诉我你的硬件参数和需求，我可以继续实现。
