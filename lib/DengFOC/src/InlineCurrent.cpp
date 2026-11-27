#include <Arduino.h>
#include "InlineCurrent.h"
#include "config.h"

/** @file InlineCurrent.cpp @brief ADC 电流采样、零偏校准及电压到电流换算实现。 */
#define _ADC_CONV ((dengfoc_config::kAdcReferenceVoltage) / (dengfoc_config::kAdcMaxCount)) //!< ADC 计数到电压
#define _isset(a) ((a) != (dengfoc_config::kAdcPinNotConnected)) //!< 判断 ADC 引脚是否有效

/**
 * @brief 根据电机编号加载 ADC 引脚、采样电阻、运放增益和方向符号。
 * @note 所有成员先赋安全初值，未连接的 C 相不会参与配置或读取。
 */
CurrSense::CurrSense(int Mot_Num)
{
  _Mot_Num = Mot_Num;
  // 初始时将所有可选引脚和状态置为无效/零值。
  pinA = dengfoc_config::kAdcPinNotConnected;
  pinB = dengfoc_config::kAdcPinNotConnected;
  pinC = dengfoc_config::kAdcPinNotConnected;
  offset_ia = 0.0f;
  offset_ib = 0.0f;
  offset_ic = 0.0f;
  current_a = 0.0f;
  current_b = 0.0f;
  current_c = 0.0f;
  _shunt_resistor = dengfoc_config::kShuntResistanceOhm;
  amp_gain = dengfoc_config::kCurrentSenseAmplifierGain;
  // 采样电压先除以采样电阻，再除以运放增益得到电流。
  volts_to_amps_ratio = 1.0f / _shunt_resistor / amp_gain;

  // 按电机编号选择 A/B 相 ADC，并应用硬件电流方向校正。
  if (Mot_Num == 0) {
    pinA = dengfoc_config::kM0CurrentPins[0];
    pinB = dengfoc_config::kM0CurrentPins[1];
    gain_a = volts_to_amps_ratio * dengfoc_config::kM0CurrentGainSign;
    gain_b = volts_to_amps_ratio * dengfoc_config::kM0CurrentGainSign;
  } else if (Mot_Num == 1) {
    pinA = dengfoc_config::kM1CurrentPins[0];
    pinB = dengfoc_config::kM1CurrentPins[1];
    gain_a = volts_to_amps_ratio * dengfoc_config::kM1CurrentGainSign;
    gain_b = volts_to_amps_ratio * dengfoc_config::kM1CurrentGainSign;
  } else {
    gain_a = 0.0f;
    gain_b = 0.0f;
  }
  gain_c = 0.0f;
}

/** @brief 读取 ADC 原始计数并转换为电压，返回单位 V。 */
float CurrSense::readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}

/** @brief 配置有效电流采样引脚为输入，跳过标记为未连接的 C 相。 */
void CurrSense::configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if (_isset(pinC)) pinMode(pinC, INPUT);
}

/**
 * @brief 累计 1000 次零电流 ADC 读数并求平均，得到各相零偏电压。
 * @note 校准时驱动器不应输出电流；每次循环延时约 1 ms。
 */
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 1000;

    // 清零各相累加器，准备计算零电流平均电压。
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读数1000次
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }
    // 求平均，得到误差
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

/** @brief 依次完成 ADC 引脚配置和零偏校准。 */
void CurrSense::init(){
    configureADCInline(pinA,pinB,pinC);
    calibrateOffsets();
}

/**
 * @brief 读取相电压、扣除零偏并换算为相电流。
 * @note 返回结果单位 A；未连接 C 相时 current_c 固定为 0。
 */
void CurrSense::getPhaseCurrents(){
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;  // A 相电流，单位 A
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;  // B 相电流，单位 A
    current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c;  // C 相电流，未连接时为 0
}
