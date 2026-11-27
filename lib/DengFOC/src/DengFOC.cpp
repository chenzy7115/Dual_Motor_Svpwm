#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h"
#include "config.h"

/**
 * @file DengFOC.cpp
 * @brief 双电机有感 FOC 核心实现：传感器、电流环、SVPWM 和多级闭环。
 */

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
float voltage_power_supply = dengfoc_config::kPowerSupplyVoltage; //!< 当前母线电压，单位 V
#define _PI 3.14159265359f       //!< 圆周率 π
#define _PI_2 1.57079632679f     //!< π/2
#define _PI_3 1.0471975512f      //!< π/3，SVPWM 每个扇区角度
#define _3PI_2 4.71238898038f    //!< 3π/2，编码器对齐方向

// 两路编码器零电角度在每次上电对齐后更新。
float S0_zero_electric_angle = 0;
float S1_zero_electric_angle = 0;
// 磁极对数和方向可由对齐接口在运行时更新。
int M0_PP = dengfoc_config::kM0PolePairs, M0_DIR = dengfoc_config::kM0Direction;
int M1_PP = dengfoc_config::kM1PolePairs, M1_DIR = dengfoc_config::kM1Direction;

// 两路电机分别使用独立的速度和电流低通滤波器。
LowPassFilter M0_Vel_Flt = LowPassFilter(dengfoc_config::kVelocityFilterTimeConstant);
LowPassFilter M1_Vel_Flt = LowPassFilter(dengfoc_config::kVelocityFilterTimeConstant);
LowPassFilter M0_Curr_Flt = LowPassFilter(dengfoc_config::kCurrentFilterTimeConstant);
LowPassFilter M1_Curr_Flt = LowPassFilter(dengfoc_config::kCurrentFilterTimeConstant);
// 每路电机拥有独立的速度环、角度环和电流环。
PIDController vel_loop_M0 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, voltage_power_supply / 2);
PIDController angle_loop_M0 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, dengfoc_config::kDefaultAngleLimit);
PIDController current_loop_M0 = PIDController(dengfoc_config::kDefaultCurrentP, dengfoc_config::kDefaultCurrentI, dengfoc_config::kDefaultCurrentD, dengfoc_config::kDefaultCurrentRamp, dengfoc_config::kDefaultCurrentLimit);
PIDController vel_loop_M1 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, voltage_power_supply / 2);
PIDController angle_loop_M1 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, dengfoc_config::kDefaultAngleLimit);
PIDController current_loop_M1 = PIDController(dengfoc_config::kDefaultCurrentP, dengfoc_config::kDefaultCurrentI, dengfoc_config::kDefaultCurrentD, dengfoc_config::kDefaultCurrentRamp, dengfoc_config::kDefaultCurrentLimit);
// 两路编码器分别绑定独立 I²C 控制器，避免共用总线造成地址/时序冲突。
Sensor_AS5600 S0 = Sensor_AS5600(0);
Sensor_AS5600 S1 = Sensor_AS5600(1);
TwoWire S0_I2C = TwoWire(dengfoc_config::kM0I2cBus);
TwoWire S1_I2C = TwoWire(dengfoc_config::kM1I2cBus);

#define _SQRT3 1.73205080757f    //!< √3，用于 SVPWM 有效矢量时间
#define _1_SQRT3 0.57735026919f  //!< 1/√3，用于 Clarke 变换
#define _2_SQRT3 1.15470053838f  //!< 2/√3，用于 Clarke 变换

// 两路电流采样对象在 DFOC_Vbus() 中配置 ADC 并校准零偏。
CurrSense CS_M0 = CurrSense(0);
CurrSense CS_M1 = CurrSense(1);

//=================PID 设置函数=================
/** @brief 更新 M0 速度环 P/I/D、输出变化率和输出限幅。 */
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit) {
  vel_loop_M0.P = P;
  vel_loop_M0.I = I;
  vel_loop_M0.D = D;
  vel_loop_M0.output_ramp = ramp;
  vel_loop_M0.limit = limit;
}
/** @brief 更新 M0 角度环 P/I/D、输出变化率和输出限幅。 */
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit) {
  angle_loop_M0.P = P;
  angle_loop_M0.I = I;
  angle_loop_M0.D = D;
  angle_loop_M0.output_ramp = ramp;
  angle_loop_M0.limit = limit;
}
/** @brief 更新 M0 电流环 P/I/D 和输出变化率，输出限幅保持不变。 */
void DFOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp) {
  current_loop_M0.P = P;
  current_loop_M0.I = I;
  current_loop_M0.D = D;
  current_loop_M0.output_ramp = ramp;
}
/** @brief 更新 M1 速度环 P/I/D、输出变化率和输出限幅。 */
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp, float limit) {
  vel_loop_M1.P = P;
  vel_loop_M1.I = I;
  vel_loop_M1.D = D;
  vel_loop_M1.output_ramp = ramp;
  vel_loop_M1.limit = limit;
}
/** @brief 更新 M1 角度环 P/I/D、输出变化率和输出限幅。 */
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit) {
  angle_loop_M1.P = P;
  angle_loop_M1.I = I;
  angle_loop_M1.D = D;
  angle_loop_M1.output_ramp = ramp;
  angle_loop_M1.limit = limit;
}
/** @brief 更新 M1 电流环 P/I/D 和输出变化率，输出限幅保持不变。 */
void DFOC_M1_SET_CURRENT_PID(float P, float I, float D, float ramp) {
  current_loop_M1.P = P;
  current_loop_M1.I = I;
  current_loop_M1.D = D;
  current_loop_M1.output_ramp = ramp;
}

/** @brief 输入 M0 速度误差，返回速度环电流目标。 */
float DFOC_M0_VEL_PID(float error) {
  return vel_loop_M0(error);
}
/** @brief 输入 M0 角度误差，返回角度环速度目标。 */
float DFOC_M0_ANGLE_PID(float error) {
  return angle_loop_M0(error);
}
/** @brief 输入 M1 速度误差，返回速度环电流目标。 */
float DFOC_M1_VEL_PID(float error) {
  return vel_loop_M1(error);
}
/** @brief 输入 M1 角度误差，返回角度环速度目标。 */
float DFOC_M1_ANGLE_PID(float error) {
  return angle_loop_M1(error);
}

/** @brief 使用 fmod 将任意角度归一化到 [0, 2π)，负角度加 2π。 */
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

/**
 * @brief 将 M0 三相相电压转换为占空比并写入 LEDC。
 * @note 相电压限制在 0..母线电压，占空比限制在 0..1。
 */
void M0_setPwm(float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // Arduino-ESP32 3.x 按 GPIO 写入，通道在 DFOC_Vbus() 中绑定。
  ledcWrite(dengfoc_config::kM0PwmPins[0], static_cast<uint32_t>(dc_a * dengfoc_config::kPwmMaxDuty));
  ledcWrite(dengfoc_config::kM0PwmPins[1], static_cast<uint32_t>(dc_b * dengfoc_config::kPwmMaxDuty));
  ledcWrite(dengfoc_config::kM0PwmPins[2], static_cast<uint32_t>(dc_c * dengfoc_config::kPwmMaxDuty));
}

/** @brief 将 M1 三相相电压转换为占空比并写入对应的三个 LEDC 通道。 */
void M1_setPwm(float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  ledcWrite(dengfoc_config::kM1PwmPins[0], static_cast<uint32_t>(dc_a * dengfoc_config::kPwmMaxDuty));
  ledcWrite(dengfoc_config::kM1PwmPins[1], static_cast<uint32_t>(dc_b * dengfoc_config::kPwmMaxDuty));
  ledcWrite(dengfoc_config::kM1PwmPins[2], static_cast<uint32_t>(dc_c * dengfoc_config::kPwmMaxDuty));
}

/**
 * @brief 根据 M0 q 轴电压和电角度计算七段式 SVPWM 占空比。
 * @param Uq q 轴电压，单位 V；负值通过反转电角度实现反向力矩
 * @param angle_el 电角度，单位 rad
 */
void M0_setTorque(float Uq, float angle_el) {
  // 负电压等效于电角度旋转 180°，随后按正电压计算。
  if (Uq < 0)
    angle_el += _PI;
  Uq = abs(Uq);

  // 源算法的坐标约定先偏移 π/2，再按每 60° 一个扇区定位。
  angle_el = _normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // 计算相邻两个有效矢量的作用时间 T1/T2，以及零矢量时间 T0。
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  // 按六个扇区分配零矢量的一半到两侧，得到 A/B/C 相占空比。
  float Ta, Tb, Tc;
  switch (sector) {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  // 占空比换算为相电压后交给 M0 PWM 输出接口。
  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  M0_setPwm(Ua, Ub, Uc);
}

/** @brief M1 的七段式 SVPWM 实现，计算流程和坐标约定与 M0 完全一致。 */
void M1_setTorque(float Uq, float angle_el) {
  if (Uq < 0)
    angle_el += _PI;
  Uq = abs(Uq);

  angle_el = _normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // 计算相邻有效矢量作用时间 T1/T2，以及零矢量时间 T0。
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  // 按六个扇区分配零矢量的一半到两侧，得到 A/B/C 相占空比。
  float Ta, Tb, Tc;
  switch (sector) {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  // 占空比换算为相电压后交给 M1 PWM 输出接口。
  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  M1_setPwm(Ua, Ub, Uc);
}

/** @brief 先配置使能 GPIO 为输出，再写入有效电平，避免上电时引脚状态不确定。 */
void DFOC_enable() {
  pinMode(dengfoc_config::kEnablePin, OUTPUT);
  digitalWrite(dengfoc_config::kEnablePin, dengfoc_config::kEnableActiveLevel);
}

/** @brief 将共用使能 GPIO 置为无效电平，关闭两路驱动器输出。 */
void DFOC_disable() {
  pinMode(dengfoc_config::kEnablePin, OUTPUT);
  digitalWrite(dengfoc_config::kEnablePin, !dengfoc_config::kEnableActiveLevel);
}

/**
 * @brief 初始化双电机运行所需的全部外设。
 * @param power_supply 实际驱动器母线电压，单位 V，用于占空比换算和速度环限幅
 * @note 两路电流采样各校准 1000 次，因此初始化会有约 2 s 延时。
 */
void DFOC_Vbus(float power_supply) {
  voltage_power_supply = power_supply;
  pinMode(dengfoc_config::kM0PwmPins[0], OUTPUT);
  pinMode(dengfoc_config::kM0PwmPins[1], OUTPUT);
  pinMode(dengfoc_config::kM0PwmPins[2], OUTPUT);
  pinMode(dengfoc_config::kM1PwmPins[0], OUTPUT);
  pinMode(dengfoc_config::kM1PwmPins[1], OUTPUT);
  pinMode(dengfoc_config::kM1PwmPins[2], OUTPUT);

  // 为两路电机绑定六个 LEDC 通道，频率和分辨率由 config.h 统一设置。
  for (int i = 0; i < 3; i++) {
    ledcAttachChannel(dengfoc_config::kM0PwmPins[i], dengfoc_config::kPwmFrequencyHz, dengfoc_config::kPwmResolutionBits, dengfoc_config::kM0PwmChannels[i]);
    ledcAttachChannel(dengfoc_config::kM1PwmPins[i], dengfoc_config::kPwmFrequencyHz, dengfoc_config::kPwmResolutionBits, dengfoc_config::kM1PwmChannels[i]);
  }
  Serial.println("完成PWM初始化设置");
  pinMode(dengfoc_config::kEnablePin, OUTPUT);

  // 使用两个独立 I²C 总线初始化两路 AS5600。
  S0_I2C.begin(dengfoc_config::kM0I2cSdaPin, dengfoc_config::kM0I2cSclPin, dengfoc_config::kI2cFrequencyHz);
  S1_I2C.begin(dengfoc_config::kM1I2cSdaPin, dengfoc_config::kM1I2cSclPin, dengfoc_config::kI2cFrequencyHz);

  S0.Sensor_init(&S0_I2C);
  S1.Sensor_init(&S1_I2C);

  Serial.println("编码器加载完毕");

  //两路速度环均使用实际母线电压的一半作为输出限幅。
  vel_loop_M0 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, voltage_power_supply / 2);
  vel_loop_M1 = PIDController(dengfoc_config::kDefaultVelocityP, dengfoc_config::kDefaultVelocityI, dengfoc_config::kDefaultVelocityD, dengfoc_config::kDefaultVelocityRamp, voltage_power_supply / 2);
  // 设置 ADC 为 12 位，然后依次校准两路电流采样零偏。
  analogReadResolution(dengfoc_config::kAdcResolutionBits);
  CS_M0.init();
  CS_M1.init();
}

/** @brief 由 M0 机械角、磁极对数、方向和零电角度计算归一化电角度。 */
float S0_electricalAngle() {
  return _normalizeAngle((float)(M0_DIR * M0_PP) * S0.getMechanicalAngle() - S0_zero_electric_angle);
}

/** @brief 由 M1 机械角、磁极对数、方向和零电角度计算归一化电角度。 */
float S1_electricalAngle() {
  return _normalizeAngle((float)(M1_DIR * M1_PP) * S1.getMechanicalAngle() - S1_zero_electric_angle);
}

/**
 * @brief 施加固定方向力矩使 M0 转子对齐，再记录零电角度。
 * @param _PP 电机磁极对数
 * @param _DIR 电机/编码器方向，1 或 -1
 */
void DFOC_M0_alignSensor(int _PP, int _DIR) {
  M0_PP = _PP;
  M0_DIR = _DIR;
  M0_setTorque(dengfoc_config::kAlignVoltage, _3PI_2);  //起劲
  delay(dengfoc_config::kAlignSettleTimeMs);
  S0.Sensor_update();
  S0_zero_electric_angle = S0_electricalAngle();
  M0_setTorque(0, _3PI_2);  //松劲（解除校准）
  Serial.print("M0 0电角度：");
  Serial.println(S0_zero_electric_angle);
}

/** @brief M1 编码器对齐流程：施加 3 V 力矩、等待稳定、记录零电角度后撤去力矩。 */
void DFOC_M1_alignSensor(int _PP, int _DIR) {
  M1_PP = _PP;
  M1_DIR = _DIR;
  M1_setTorque(dengfoc_config::kAlignVoltage, _3PI_2);  //起劲
  delay(dengfoc_config::kAlignSettleTimeMs);
  S1.Sensor_update();
  S1_zero_electric_angle = S1_electricalAngle();
  M1_setTorque(0, _3PI_2);  //松劲（解除校准）
  Serial.print("M1 0电角度：");
  Serial.println(S1_zero_electric_angle);
}

/** @brief 获取 M0 按方向修正后的累计机械角度，单位 rad。 */
float DFOC_M0_Angle() {
  return M0_DIR * S0.getAngle();
}
/** @brief 获取 M1 按方向修正后的累计机械角度，单位 rad。 */
float DFOC_M1_Angle() {
  return M1_DIR * S1.getAngle();
}

/**
 * @brief 使用两相电流执行 Clarke/Park 变换并返回 q 轴电流。
 * @param current_a A 相电流，单位 A
 * @param current_b B 相电流，单位 A
 * @param angle_el 当前电角度，单位 rad
 * @return q 轴电流 Iq，单位 A
 */
float cal_Iq_Id(float current_a, float current_b, float angle_el) {
  // 两相电流转换到 α/β 静止坐标系。
  float I_alpha = current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  // Park 变换到随转子旋转的 d/q 坐标系，本工程只使用 q 轴电流。
  float ct = cos(angle_el);
  float st = sin(angle_el);
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

/** @brief 获取 M0 原始 Iq 并经过电流低通滤波，返回单位 A。 */
float DFOC_M0_Current() {
  float I_q_M0_ori = cal_Iq_Id(CS_M0.current_a, CS_M0.current_b, S0_electricalAngle());
  float I_q_M0_flit = M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;
}

/** @brief 获取 M1 原始 Iq 并经过电流低通滤波，返回单位 A。 */
float DFOC_M1_Current() {
  float I_q_M1_ori = cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, S1_electricalAngle());
  float I_q_M1_flit = M1_Curr_Flt(I_q_M1_ori);
  return I_q_M1_flit;
}

/** @brief 获取 M0 角速度并按电机方向修正后低通滤波，返回单位 rad/s。 */
float DFOC_M0_Velocity() {
  float vel_M0_ori = S0.getVelocity();
  float vel_M0_flit = M0_Vel_Flt(M0_DIR * vel_M0_ori);
  return vel_M0_flit;
}

/** @brief 获取 M1 角速度并按电机方向修正后低通滤波，返回单位 rad/s。 */
float DFOC_M1_Velocity() {
  float vel_M1_ori = S1.getVelocity();
  float vel_M1_flit = M1_Vel_Flt(M1_DIR * vel_M1_ori);
  return vel_M1_flit;
}

//==============串口接收==============
float M0_target;   //!< 当前共享力矩目标，单位 A
float M1_target;   //!< 源例程保留的 M1 目标变量，当前控制流程未使用
int commaPosition; //!< 换行符在命令字符串中的位置

/**
 * @brief 累积串口字符，遇到换行符时解析一个浮点力矩目标。
 * @return 本次接收到的完整命令；没有完整命令时返回空字符串
 * @note 源例程只接受单个数值，例如 "0.5\n"，不支持逗号分隔的双电机目标。
 */
String serialReceiveUserCommand() {
  static String received_chars;
  String command = "";

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    received_chars += inChar;

    // 换行表示一条命令结束，先保存完整字符串再解析。
    if (inChar == '\n') {
      command = received_chars;

      commaPosition = command.indexOf('\n');
      if (commaPosition != -1) {
        M0_target = command.substring(0, commaPosition).toDouble();
        Serial.println(M0_target);
      }
      received_chars = "";
    }
  }
  return command;
}

/** @brief 返回串口设置的共享电流目标，单位 A。 */
float serial_motor_target() {
  return M0_target;
}

//================简易接口函数================
/** @brief M0 电流闭环：目标电流减去反馈 Iq，经 PID 得到 q 轴电压后执行 SVPWM。 */
void DFOC_M0_setTorque(float Target)  //电流力矩环
{
  M0_setTorque(current_loop_M0(Target - DFOC_M0_Current()), S0_electricalAngle());
}

/** @brief M1 电流闭环：目标电流减去反馈 Iq，经 PID 得到 q 轴电压后执行 SVPWM。 */
void DFOC_M1_setTorque(float Target)  //电流力矩环
{
  M1_setTorque(current_loop_M1(Target - DFOC_M1_Current()), S1_electricalAngle());
}

/** @brief M0 位置-速度-电流三级闭环；位置 rad 转角度误差后逐级生成速度/电流目标。 */
void DFOC_M0_set_Velocity_Angle(float Target)  //角度-速度-力 位置闭环
{
  DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI) - DFOC_M0_Velocity()));
}

/** @brief M1 位置-速度-电流三级闭环；位置 rad 转角度误差后逐级生成速度/电流目标。 */
void DFOC_M1_set_Velocity_Angle(float Target)  //角度-速度-力 位置闭环
{
  DFOC_M1_setTorque(DFOC_M1_VEL_PID(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI) - DFOC_M1_Velocity()));
}

/** @brief M0 速度-电流闭环；将 rad/s 误差换算为角度误差后交给速度 PID。 */
void DFOC_M0_setVelocity(float Target)  //速度闭环
{
  DFOC_M0_setTorque(DFOC_M0_VEL_PID((Target - DFOC_M0_Velocity()) * 180 / PI));
}

/** @brief M1 速度-电流闭环；将 rad/s 误差换算为角度误差后交给速度 PID。 */
void DFOC_M1_setVelocity(float Target)  //速度闭环
{
  DFOC_M1_setTorque(DFOC_M1_VEL_PID((Target - DFOC_M1_Velocity()) * 180 / PI));
}

/** @brief M0 位置-电流闭环；角度 PID 输出直接作为电流目标。 */
void DFOC_M0_set_Force_Angle(float Target)  //力位闭环
{
  DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target - DFOC_M0_Angle()) * 180 / PI));
}

/** @brief M1 位置-电流闭环；角度 PID 输出直接作为电流目标。 */
void DFOC_M1_set_Force_Angle(float Target)  //力位闭环
{
  DFOC_M1_setTorque(DFOC_M1_ANGLE_PID((Target - DFOC_M1_Angle()) * 180 / PI));
}

/** @brief 每个 FOC 循环先更新两路编码器，再读取两路 A/B 相电流。 */
void runFOC() {
  //====传感器更新====
  S0.Sensor_update();
  S1.Sensor_update();
  CS_M0.getPhaseCurrents();
  CS_M1.getPhaseCurrents();
}
