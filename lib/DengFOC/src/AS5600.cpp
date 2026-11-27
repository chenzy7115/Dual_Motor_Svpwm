#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>
#include "config.h"

/** @file AS5600.cpp @brief AS5600 寄存器读取、圈数累计和速度计算实现。 */
#define _2PI 6.28318530718f  //!< 一整圈对应的弧度值

/**
 * @brief 读取 AS5600 的 12 位角度并转换为 [0, 2π)。
 * @return 当前单圈机械角度，单位 rad
 */
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C;  //!< 角度寄存器起始地址，0x0C 为高字节

  byte readArray[2];              //!< 依次保存角度高字节和低字节
  uint16_t readValue = 0;

  // 先写入寄存器地址，再以重复起始条件连续读取两个字节。
  wire->beginTransmission(dengfoc_config::kAs5600I2cAddress);
  wire->write(angle_reg_msb);
  wire->endTransmission(false);

  wire->requestFrom(dengfoc_config::kAs5600I2cAddress, (uint8_t)2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }
  // AS5600 输出 12 位角度：高字节使用低 4 位，低字节使用全部 8 位。
  int _bit_resolution=12;
  int _bits_used_msb=11-7;
  float cpr = pow(2, _bit_resolution);       //!< 每圈原始计数，12 位为 4096
  int lsb_used = _bit_resolution - _bits_used_msb;

  uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
  uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );

  readValue = ( readArray[1] &  lsb_mask );
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
  // 将 0..4095 的计数线性映射到 0..2π。
  return (readValue/ (float)cpr) * _2PI;
}

/** @brief 保存电机编号，具体 I²C 引脚在初始化时由外部总线传入。 */
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;
}

/**
 * @brief 绑定 I²C 并等待编码器稳定，同时建立角度和速度的初始基准。
 * @param _wire 已配置 SDA/SCL/频率的 TwoWire 总线
 */
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   // 已初始化的总线重复 begin 不会改变其引脚配置
    delay(500);      // 等待 AS5600 上电稳定
    getSensorAngle();
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle();
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle();
    delayMicroseconds(1);
    angle_prev = getSensorAngle();
    angle_prev_ts = micros();
}
/** @brief 更新单圈角度，并在跨过 0/2π 边界时修正累计圈数。 */
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();
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
float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
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
