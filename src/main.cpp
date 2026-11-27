//DengFOC V0.2
//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种自由软件许可协议，常用于GNU计划的软件和其他软件。

#include <Arduino.h>
#include "DengFOC.h"
#include "config.h"

/**
 * @brief ESP32 上电初始化。
 * @note 先使能驱动器，再初始化 PWM、两路编码器和电流采样，最后执行机械对齐。
 *       对齐过程中电机会转动，使用前应确认机械机构允许自由运动。
 */
void setup() {
  // 串口同时用于接收力矩目标和发送电流、角度、速度数据。
  Serial.begin(dengfoc_config::kSerialBaudRate);

  DFOC_enable();  // 必须先配置并拉高驱动器使能引脚。
  DFOC_Vbus(dengfoc_config::kPowerSupplyVoltage);  // 设置实际母线电压，单位 V。

  // 依次对齐两个电机；参数依次为磁极对数和编码器方向。
  DFOC_M0_alignSensor(dengfoc_config::kM0PolePairs, dengfoc_config::kM0Direction);
  DFOC_M1_alignSensor(dengfoc_config::kM1PolePairs, dengfoc_config::kM1Direction);
}

// 串口上报循环计数，避免每个 FOC 循环都输出数据。
int count = 0;

/**
 * @brief 双电机电流力矩控制主循环。
 * @note 每次循环先更新传感器，再计算两路电流环输出；有效目标单位为 A。
 */
void loop() {
  // 更新两路编码器角度及 A/B 相电流，后续控制接口使用这些最新数据。
  runFOC();
  // DFOC_M0_setTorque(1);
  // DFOC_M1_setTorque(1);

  // 以下为源例程保留的可选控制模式；默认只启用最后的电流力矩模式。

  // 力位模式：角度误差经过角度 PID 后作为电流目标，目标单位为 rad。
  // DFOC_M0_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);
  // DFOC_M0_SET_CURRENT_PID(1.25,50,0,100000);
  // DFOC_M0_set_Force_Angle(serial_motor_target());
  // DFOC_M1_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);
  // DFOC_M1_SET_CURRENT_PID(1.25,50,0,100000);
  // DFOC_M1_set_Force_Angle(serial_motor_target());

  // 速度模式：速度误差经过速度 PID 后作为电流目标，目标单位为 rad/s。
  // DFOC_M0_SET_VEL_PID(3,2,0,100000,0.5);
  // DFOC_M0_SET_CURRENT_PID(0.5,50,0,100000);
  // DFOC_M0_setVelocity(serial_motor_target());
  // DFOC_M1_SET_VEL_PID(0.1,2,0,100000,0.5);
  // DFOC_M1_SET_CURRENT_PID(0.5,50,0,100000);
  // DFOC_M1_setVelocity(serial_motor_target());

  // 位置-速度-电流三级闭环：位置单位为 rad，速度单位为 rad/s。
  // DFOC_M0_SET_ANGLE_PID(1, 0, 0, 100000, 30);
  // DFOC_M0_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
  // DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000);
  // DFOC_M0_set_Velocity_Angle(serial_motor_target());

  // 位置-速度-电流三级闭环：位置单位为 rad，速度单位为 rad/s。
  // DFOC_M1_SET_ANGLE_PID(1, 0, 0, 100000, 30);
  // DFOC_M1_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
  // DFOC_M1_SET_CURRENT_PID(5, 200, 0, 100000);
  // DFOC_M1_set_Velocity_Angle(serial_motor_target());

  // 电流力矩模式：P、I、D、输出变化率限制，参数与源例程一致。
  DFOC_M1_SET_CURRENT_PID(dengfoc_config::kCurrentPidP, dengfoc_config::kCurrentPidI, dengfoc_config::kCurrentPidD, dengfoc_config::kCurrentPidRamp);
  DFOC_M0_SET_CURRENT_PID(dengfoc_config::kCurrentPidP, dengfoc_config::kCurrentPidI, dengfoc_config::kCurrentPidD, dengfoc_config::kCurrentPidRamp);

  // 源例程只解析一个串口目标，并让两个电机使用同一个力矩目标。
  DFOC_M0_setTorque(serial_motor_target());
  DFOC_M1_setTorque(serial_motor_target());

  // 每 101 个控制循环输出一次：M0 电流、M1 电流、M0 累计角度、M0 速度、目标。
  count++;
  if (count > dengfoc_config::kSerialReportIntervalLoops) {
    count = 0;
    Serial.printf("%f,%f,%f,%f,%f\n", DFOC_M0_Current(), DFOC_M1_Current(), DFOC_M0_Angle(), DFOC_M0_Velocity(), serial_motor_target());
  }

  // 接收以换行符结尾的串口命令，例如发送 "0.5\n" 设置 0.5 A 目标。
  serialReceiveUserCommand();
}