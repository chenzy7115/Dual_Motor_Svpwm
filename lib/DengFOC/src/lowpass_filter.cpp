#include "lowpass_filter.h"

/** @file lowpass_filter.cpp @brief 一阶低通滤波器的初始化和实时计算。 */

/** @brief 保存时间常数，将上次输出和时间戳初始化为零/当前时刻。 */
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant > 0.0f ? time_constant : 1e-3f)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}


/** @brief 重置滤波输出和时间戳。 */
void LowPassFilter::reset(float initial_value){
    y_prev = isfinite(initial_value) ? initial_value : 0.0f;
    timestamp_prev = micros();
}

/**
 * @brief 根据实际采样间隔执行一阶低通滤波。
 * @param x 当前输入值
 * @return 滤波输出；长时间无采样时直接跟随输入以恢复响应
 */
float LowPassFilter::operator() (float x)
{
    if (!isfinite(x)) {
        reset();
        return 0.0f;
    }

    // 计算距上次调用的时间，单位由 us 转换为 s。
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;

    // 修正异常负时间；暂停超过 0.3 s 时重置历史状态。
    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    // 一阶离散低通：alpha 越大，输出越平滑但延迟越高。
    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}
