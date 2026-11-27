#ifndef INLINE_CURRENT_H
#define INLINE_CURRENT_H

#include <Arduino.h>

/**
 * @file InlineCurrent.h
 * @brief 双路低侧采样电流检测，支持两相或三相 ADC 输入。
 * @note 当前工程只连接 A/B 相，C 相引脚由 config.h 标记为未连接。
 */
class CurrSense
{
  public:
    /** @brief 构造电流采样对象；Mot_Num 为 0/1 时选择 M0/M1 配置。 */
    CurrSense(int Mot_Num);
    /** @brief 读取指定 ADC 引脚并转换为电压，返回单位 V。 */
    float readADCVoltageInline(const int pinA);
    /** @brief 将有效 ADC 引脚配置为输入；无效引脚会被跳过。 */
    void configureADCInline(const int pinA,const int pinB, const int pinC);
    /** @brief 在零电流状态下采样 1000 次，计算各相 ADC 零偏电压。 */
    void calibrateOffsets();
    /** @brief 配置 ADC 引脚并执行零偏校准。 */
    void init();
    /** @brief 读取各相电压、减去零偏并换算为电流，单位 A。 */
    void getPhaseCurrents();

    float current_a;              //!< A 相电流，单位 A
    float current_b;              //!< B 相电流，单位 A
    float current_c;              //!< C 相电流，未连接时保持 0
    int pinA;                     //!< A 相 ADC GPIO
    int pinB;                     //!< B 相 ADC GPIO
    int pinC;                     //!< C 相 ADC GPIO，未连接时为 kAdcPinNotConnected
    float offset_ia;              //!< A 相零偏电压，单位 V
    float offset_ib;              //!< B 相零偏电压，单位 V
    float offset_ic;              //!< C 相零偏电压，单位 V
    float _shunt_resistor;        //!< 采样电阻，单位 Ω
    float amp_gain;               //!< 电流检测运放增益倍数
    float volts_to_amps_ratio;    //!< 电压到电流的换算系数，1/(R_shunt×Gain)
    float gain_a;                 //!< A 相最终换算系数，包含方向符号
    float gain_b;                 //!< B 相最终换算系数，包含方向符号
    float gain_c;                 //!< C 相最终换算系数，未连接时为 0
  private:
    int _Mot_Num;                 //!< 电机编号
};

#endif
