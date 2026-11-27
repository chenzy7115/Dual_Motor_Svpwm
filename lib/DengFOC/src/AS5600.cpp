#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>
#include "config.h"

/** @file AS5600.cpp @brief AS5600 寄存器读取、圈数累计和速度计算实现。 */
#define _2PI 6.28318530718f  //!< 一整圈对应的弧度值

/** @brief 记录一次通信失败，连续失败达到配置阈值后锁存故障。 */
void Sensor_AS5600::recordReadFailure() {
  if (consecutive_errors < 255) consecutive_errors++;
  if (consecutive_errors >= dengfoc_config::kEncoderMaxConsecutiveErrors) {
    fault_latched = true;
  }
}

/**
 * @brief 执行带返回值检查的 I²C 角度读取。
 * @param angle 读取成功时写入单圈机械角度，单位 rad
 * @return 通信成功且角度有效返回 true
 */
bool Sensor_AS5600::readAngleRad(float& angle) {
  if (wire == nullptr) {
    recordReadFailure();
    return false;
  }

  // 先写入 0x0C 寄存器地址，再以重复起始条件连续读取两个字节。
  wire->beginTransmission(dengfoc_config::kAs5600I2cAddress);
  wire->write((uint8_t)0x0C);
  if (wire->endTransmission(false) != 0) {
    recordReadFailure();
    return false;
  }

  if (wire->requestFrom(dengfoc_config::kAs5600I2cAddress, (uint8_t)2) != 2 ||
      wire->available() < 2) {
    while (wire->available()) wire->read();
    recordReadFailure();
    return false;
  }

  const uint8_t high_byte = wire->read();
  const uint8_t low_byte = wire->read();
  // AS5600 角度为 12 位：高字节低 4 位 + 低字节全部 8 位。
  const uint16_t raw_angle = (uint16_t(high_byte & 0x0F) << 8) | low_byte;
  angle = (float(raw_angle) / 4096.0f) * _2PI;
  if (!isfinite(angle)) {
    recordReadFailure();
    return false;
  }

  consecutive_errors = 0;
  return true;
}

/**
 * @brief 读取 AS5600 的 12 位角度并转换为 [0, 2π)。
 * @return 当前单圈机械角度，单位 rad；通信失败返回 NAN
 */
double Sensor_AS5600::getSensorAngle() {
  float angle = 0.0f;
  return readAngleRad(angle) ? angle : NAN;
}

/** @brief 保存电机编号，具体 I²C 引脚在初始化时由外部总线传入。 */
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;
}

/**
 * @brief 绑定 I²C 并等待编码器稳定，同时建立角度和速度的初始基准。
 * @param _wire 已配置 SDA/SCL/频率的 TwoWire 总线
 */
bool Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    consecutive_errors = 0;
    fault_latched = false;
    sensor_ready = false;
    wire = _wire;
    if (wire == nullptr) {
      fault_latched = true;
      return false;
    }

    wire->begin();   // 已初始化的总线重复 begin 不会改变其引脚配置
    delay(500);      // 等待 AS5600 上电稳定

    float first_angle = 0.0f;
    if (!readAngleRad(first_angle)) {
      fault_latched = true;
      return false;
    }

    angle_prev = first_angle;
    vel_angle_prev = first_angle;
    angle_prev_ts = micros();
    vel_angle_prev_ts = angle_prev_ts;
    sensor_ready = true;
    return true;
}
/** @brief 更新单圈角度，并在跨过 0/2π 边界时修正累计圈数。 */
void Sensor_AS5600::Sensor_update() {
    float val = 0.0f;
    if (!readAngleRad(val)) {
      // 读取失败时保留上次有效角度，避免错误数据污染圈数和电角度。
      return;
    }

    angle_prev_ts = micros();
    float d_angle = val - angle_prev;
    // 正常单步角差远小于 0.8 圈；超过该阈值说明发生了正/反向跨零。
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
    angle_prev = val;
}

/** @brief 返回最近一次更新的单圈机械角度，单位 rad。 */
float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

/** @brief 返回累计圈数与单圈角度之和，单位 rad。 */
double Sensor_AS5600::getAngle(){
    return double(full_rotations) * _2PI + angle_prev;
}

/**
 * @brief 使用上次速度计算以来的角度增量及时间差计算角速度。
 * @return 机械角速度，单位 rad/s
 */
float Sensor_AS5600::getVelocity() {
    // 时间戳由 us 转换为 s；异常时间间隔使用 1 ms 避免除零。
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    if(Ts <= 0) Ts = 1e-3f;
    // 角度增量同时包含完整圈数差和当前单圈角差。
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
    // 保存本次角度基准，供下一次速度计算使用。
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
