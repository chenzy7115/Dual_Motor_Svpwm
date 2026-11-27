//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种自由软件许可协议，常用于GNU计划的软件和其他软件。

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <Arduino.h>

/**
 * @file lowpass_filter.h
 * @brief 一阶低通滤波器，根据实际时间间隔自动更新滤波系数。
 */
class LowPassFilter
{
public:
    /**
     * @brief 构造低通滤波器。
     * @param Tf 滤波时间常数，单位 s；越大输出越平滑、延迟越高
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    /**
     * @brief 输入一个新采样值并返回滤波结果。
     * @param x 本次输入采样值
     * @return 低通滤波后的输出值
     */
    float operator() (float x);
    float Tf; //!< 低通滤波时间常数，单位 s

protected:
    unsigned long timestamp_prev;  //!< 上次执行时间戳，单位 us
    float y_prev; //!< 上一次滤波输出
};

#endif
