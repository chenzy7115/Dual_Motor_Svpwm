#pragma once

#include <Arduino.h>

/**
 * @file config.h
 * @brief 双电机工程的硬件、控制和通信参数。
 * @note 修改接线或电机参数时优先修改本文件；所有引脚均为 ESP32 GPIO 编号。
 */
namespace dengfoc_config {

// 驱动器与电机
// GPIO12 是 ESP32 的 strapping 引脚（MTDI），硬件上必须保证复位期间为低电平。
constexpr int kEnablePin = 12;                    //!< 两路驱动器共用使能 GPIO
constexpr uint8_t kEnableActiveLevel = HIGH;       //!< 使能有效电平；低有效时改为 LOW
constexpr float kPowerSupplyVoltage = 12.6f;       //!< 驱动器母线电压，单位 V
constexpr int kM0PolePairs = 7;                    //!< M0 电机磁极对数
constexpr int kM0Direction = 1;                    //!< M0 编码器/电机方向符号，1 或 -1
constexpr int kM1PolePairs = 7;                    //!< M1 电机磁极对数
constexpr int kM1Direction = 1;                    //!< M1 编码器/电机方向符号，1 或 -1
constexpr float kAlignVoltage = 3.0f;              //!< 编码器对齐时施加的 q 轴电压，单位 V
constexpr uint32_t kAlignSettleTimeMs = 1000;      //!< 对齐后等待转子稳定的时间，单位 ms
constexpr float kAlignMaxMovementRad = 6.28318530718f; //!< 对齐期间允许的最大机械角变化，1 圈

// 六路 LEDC PWM：通道、频率、精度均与源例程一致。
constexpr int kM0PwmPins[3] = {32, 33, 25};       //!< M0 A/B/C 相 PWM GPIO
constexpr uint8_t kM0PwmChannels[3] = {0, 1, 2};   //!< M0 A/B/C 相 LEDC 通道
constexpr int kM1PwmPins[3] = {26, 27, 14};       //!< M1 A/B/C 相 PWM GPIO
constexpr uint8_t kM1PwmChannels[3] = {3, 4, 5};   //!< M1 A/B/C 相 LEDC 通道
constexpr uint32_t kPwmFrequencyHz = 30000;       //!< PWM 开关频率，单位 Hz
constexpr uint8_t kPwmResolutionBits = 8;         //!< PWM 占空比分辨率，单位 bit
constexpr uint32_t kPwmMaxDuty = (1UL << kPwmResolutionBits) - 1; //!< 最大 PWM 计数值

// 两路 AS5600 使用独立 I²C 总线。
constexpr uint8_t kM0I2cBus = 0;                //!< M0 编码器使用的 I²C 控制器编号
constexpr int kM0I2cSdaPin = 19;                  //!< M0 编码器 SDA GPIO
constexpr int kM0I2cSclPin = 18;                  //!< M0 编码器 SCL GPIO
constexpr uint8_t kM1I2cBus = 1;                //!< M1 编码器使用的 I²C 控制器编号
constexpr int kM1I2cSdaPin = 23;                  //!< M1 编码器 SDA GPIO
constexpr int kM1I2cSclPin = 5;                   //!< M1 编码器 SCL GPIO
constexpr uint8_t kAs5600I2cAddress = 0x36;       //!< 两路 AS5600 的 7 位 I²C 地址
constexpr uint32_t kI2cFrequencyHz = 400000;     //!< 编码器 I²C 时钟，单位 Hz
constexpr uint8_t kEncoderMaxConsecutiveErrors = 20; //!< 单路编码器连续读取失败上限

// 两路驱动器均只采样 A/B 相电流，C 相保持无效状态。
constexpr int kM0CurrentPins[2] = {39, 36};      //!< M0 A/B 相电流采样 ADC GPIO
constexpr int kM1CurrentPins[2] = {35, 34};      //!< M1 A/B 相电流采样 ADC GPIO
constexpr int kAdcPinNotConnected = -1;          //!< 未连接 ADC 引脚的标记值
constexpr float kAdcReferenceVoltage = 3.3f;     //!< ESP32 ADC 参考电压，单位 V
constexpr uint8_t kAdcResolutionBits = 12;       //!< ADC 分辨率，单位 bit
constexpr float kAdcMaxCount = 4095.0f;          //!< 12 位 ADC 最大原始计数
constexpr float kShuntResistanceOhm = 0.01f;     //!< 电流采样电阻，单位 Ω
constexpr float kCurrentSenseAmplifierGain = 50.0f; //!< 电流检测运放增益倍数
constexpr float kM0CurrentGainSign = 1.0f;       //!< M0 电流方向校正，测量反向时改为 -1
constexpr float kM1CurrentGainSign = 1.0f;       //!< M1 电流方向校正，测量反向时改为 -1

// 控制环与滤波参数
constexpr float kDefaultVelocityP = 2.0f;        //!< 默认速度环比例增益
constexpr float kDefaultVelocityI = 0.0f;        //!< 默认速度环积分增益
constexpr float kDefaultVelocityD = 0.0f;        //!< 默认速度环微分增益
constexpr float kDefaultVelocityRamp = 100000.0f; //!< 默认速度环输出变化率
constexpr float kDefaultAngleLimit = 100.0f;      //!< 默认角度环输出限幅
constexpr float kSvpwmLinearModulationRatio = 0.57735026919f; //!< SVPWM 线性区上限 Vbus/√3
constexpr float kDefaultCurrentP = 1.2f;         //!< 初始化时的电流环比例增益
constexpr float kDefaultCurrentI = 0.0f;         //!< 初始化时的电流环积分增益
constexpr float kDefaultCurrentD = 0.0f;         //!< 初始化时的电流环微分增益
constexpr float kDefaultCurrentRamp = 100000.0f; //!< 初始化时的电流环输出变化率
constexpr float kVelocityFilterTimeConstant = 0.01f; //!< 速度低通时间常数，单位 s
constexpr float kCurrentFilterTimeConstant = 0.01f; //!< 电流低通时间常数，单位 s

// 入口程序的有效控制参数，保留源例程设置。
constexpr float kCurrentPidP = 5.0f;             //!< 主程序实际使用的电流环比例增益
constexpr float kCurrentPidI = 200.0f;           //!< 主程序实际使用的电流环积分增益
constexpr float kCurrentPidD = 0.0f;             //!< 主程序实际使用的电流环微分增益
constexpr float kCurrentPidRamp = 100000.0f;    //!< 主程序实际使用的电流环输出变化率

// 串口与控制安全
constexpr uint32_t kSerialBaudRate = 115200;    //!< 串口波特率，单位 bit/s
constexpr uint32_t kSerialReportIntervalLoops = 100; //!< 串口数据上报循环间隔；实际为间隔值加 1
constexpr uint8_t kSerialCommandBufferSize = 64; //!< 串口命令固定缓冲长度，单位字节
constexpr bool kSerialCommandTimeoutEnabled = false; //!< 串口超时保护；false 时目标保持到新命令或发送 0
constexpr uint32_t kSerialCommandTimeoutMs = 500; //!< 启用串口超时保护后的超时时间，单位 ms
constexpr float kMaxTargetCurrentA = 3.0f;       //!< 串口目标电流绝对限幅，单位 A
constexpr float kOverCurrentThresholdA = 4.5f;  //!< 原始相电流软件过流阈值，单位 A
constexpr uint8_t kOverCurrentDebounceCycles = 3; //!< 过流连续超阈值次数，避免瞬时误触发
constexpr uint32_t kControlPeriodUs = 1000;     //!< 主控制循环固定周期，单位 us

}  // namespace dengfoc_config
