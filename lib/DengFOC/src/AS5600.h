#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include "Wire.h"

/**
 * @file AS5600.h
 * @brief AS5600 磁编码器驱动，支持单圈读取、圈数累计和速度计算。
 */
class Sensor_AS5600
{
  public:
    /** @brief 构造编码器对象；Mot_Num 用于区分 M0/M1，当前引脚由 config.h 管理。 */
    Sensor_AS5600(int Mot_Num);
    /** @brief 绑定 I²C 总线并初始化角度、时间戳；默认使用全局 Wire。 */
    void Sensor_init(TwoWire* _wire = &Wire);
    /** @brief 读取单圈角度并检测跨零圈数，应在每个 FOC 循环调用。 */
    void Sensor_update();
    /** @brief 获取累计机械角度，单位 rad。 */
    float getAngle();
    /** @brief 根据最近两次更新的角度差计算速度，单位 rad/s。 */
    float getVelocity();
    /** @brief 获取当前单圈机械角度，范围 [0, 2π)，单位 rad。 */
    float getMechanicalAngle();
    /** @brief 从 AS5600 的 0x0C/0x0D 寄存器读取 12 位原始角度并转换为 rad。 */
    double getSensorAngle();
  private:
    int _Mot_Num;                         //!< 电机编号，0 表示 M0，1 表示 M1
    float angle_prev=0;                   //!< 上次读取的单圈角度，用于累计圈数
    long angle_prev_ts=0;                  //!< 上次角度更新时刻，单位 us
    float vel_angle_prev=0;                //!< 上次计算速度时保存的单圈角度
    long vel_angle_prev_ts=0;              //!< 上次计算速度的时刻，单位 us
    int32_t full_rotations=0;              //!< 累计完整圈数
    int32_t vel_full_rotations=0;          //!< 上次计算速度时的累计圈数
    TwoWire* wire = nullptr;              //!< 当前编码器使用的 I²C 总线
};

#endif
