//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种自由软件许可协议，常用于GNU计划的软件和其他软件。

#ifndef DENGFOC_H
#define DENGFOC_H

#include <Arduino.h>

/**
 * @file DengFOC.h
 * @brief 双电机有感 FOC 与 SVPWM 公共接口。
 * @note 角度默认使用 rad，速度使用 rad/s，电流及力矩电压使用 A/V。
 */

// ---------- 底层 PWM 与驱动器 ----------
/** @brief 将 M0 三相相电压写入 PWM；输入单位为 V。 */
void M0_setPwm(float Ua, float Ub, float Uc);
/** @brief 将 M1 三相相电压写入 PWM；输入单位为 V。 */
void M1_setPwm(float Ua, float Ub, float Uc);
/** @brief 使能驱动器，并先配置使能引脚为输出。 */
void DFOC_enable();
/** @brief 禁用驱动器输出。 */
void DFOC_disable();

// ---------- SVPWM 底层力矩接口 ----------
/** @brief 用 M0 电角度和 q 轴电压执行 SVPWM；Uq 单位为 V。 */
void M0_setTorque(float Uq, float angle_el);
/** @brief 用 M1 电角度和 q 轴电压执行 SVPWM；Uq 单位为 V。 */
void M1_setTorque(float Uq, float angle_el);

// ---------- 数学、初始化与传感器 ----------
/** @brief 将任意角度归一化到 [0, 2π)。 */
float _normalizeAngle(float angle);
/** @brief 设置母线电压并初始化 PWM、I²C、编码器和 ADC 电流采样。 */
void DFOC_Vbus(float power_supply);
/** @brief 对齐 M0 编码器；参数为磁极对数和方向。 */
void DFOC_M0_alignSensor(int _PP, int _DIR);
/** @brief 对齐 M1 编码器；参数为磁极对数和方向。 */
void DFOC_M1_alignSensor(int _PP, int _DIR);
/** @brief 获取 M0 归一化电角度，单位 rad。 */
float S0_electricalAngle();
/** @brief 获取 M1 归一化电角度，单位 rad。 */
float S1_electricalAngle();
/** @brief 由 A/B 相电流和电角度计算 q 轴电流，返回单位 A。 */
float cal_Iq_Id(float current_a, float current_b, float angle_el);

// ---------- 串口 ----------
/** @brief 获取当前串口力矩目标，单位 A。 */
float serial_motor_target();
/** @brief 读取换行结尾的串口命令，并返回本次完整命令。 */
String serialReceiveUserCommand();

// ---------- 传感器数据 ----------
/** @brief 获取 M0 滤波后的速度，单位 rad/s。 */
float DFOC_M0_Velocity();
/** @brief 获取 M1 滤波后的速度，单位 rad/s。 */
float DFOC_M1_Velocity();
/** @brief 获取 M0 累计机械角度，单位 rad。 */
float DFOC_M0_Angle();
/** @brief 获取 M1 累计机械角度，单位 rad。 */
float DFOC_M1_Angle();
/** @brief 获取 M0 滤波后的 q 轴电流，单位 A。 */
float DFOC_M0_Current();
/** @brief 获取 M1 滤波后的 q 轴电流，单位 A。 */
float DFOC_M1_Current();

// ---------- PID 参数设置 ----------
/** @brief 设置 M0 角度 PID；ramp 为输出变化率，limit 为输出限幅。 */
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit);
/** @brief 设置 M0 速度 PID；ramp 为输出变化率，limit 为输出限幅。 */
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp, float limit);
/** @brief 设置 M0 电流 PID；输出限幅沿用初始化值。 */
void DFOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp);
/** @brief 设置 M1 角度 PID；ramp 为输出变化率，limit 为输出限幅。 */
void DFOC_M1_SET_ANGLE_PID(float P, float I, float D, float ramp, float limit);
/** @brief 设置 M1 速度 PID；ramp 为输出变化率，limit 为输出限幅。 */
void DFOC_M1_SET_VEL_PID(float P, float I, float D, float ramp, float limit);
/** @brief 设置 M1 电流 PID；输出限幅沿用初始化值。 */
void DFOC_M1_SET_CURRENT_PID(float P, float I, float D, float ramp);

// ---------- PID 计算接口 ----------
/** @brief 根据 M0 速度误差计算电流目标，返回单位 A。 */
float DFOC_M0_VEL_PID(float error);
/** @brief 根据 M0 角度误差计算速度目标，返回单位 rad/s。 */
float DFOC_M0_ANGLE_PID(float error);
/** @brief 根据 M1 速度误差计算电流目标，返回单位 A。 */
float DFOC_M1_VEL_PID(float error);
/** @brief 根据 M1 角度误差计算速度目标，返回单位 rad/s。 */
float DFOC_M1_ANGLE_PID(float error);

// ---------- 闭环控制接口 ----------
/** @brief M0 位置-速度-电流三级闭环，目标位置单位 rad。 */
void DFOC_M0_set_Velocity_Angle(float Target);
/** @brief M0 速度-电流闭环，目标速度单位 rad/s。 */
void DFOC_M0_setVelocity(float Target);
/** @brief M0 位置-电流闭环，目标位置单位 rad。 */
void DFOC_M0_set_Force_Angle(float Target);
/** @brief M1 位置-速度-电流三级闭环，目标位置单位 rad。 */
void DFOC_M1_set_Velocity_Angle(float Target);
/** @brief M1 速度-电流闭环，目标速度单位 rad/s。 */
void DFOC_M1_setVelocity(float Target);
/** @brief M1 位置-电流闭环，目标位置单位 rad。 */
void DFOC_M1_set_Force_Angle(float Target);
/** @brief M0 电流闭环，目标 q 轴电流单位 A。 */
void DFOC_M0_setTorque(float Target);
/** @brief M1 电流闭环，目标 q 轴电流单位 A。 */
void DFOC_M1_setTorque(float Target);

/** @brief 每个控制循环调用一次，更新两路编码器和电流采样。 */
void runFOC();

#endif
