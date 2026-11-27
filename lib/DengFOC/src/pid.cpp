#include "pid.h"
#include <Arduino.h>

/** @file pid.cpp @brief 带实际时间步长、梯形积分、限幅和变化率限制的 PID 实现。 */
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/** @brief 保存 PID 参数并将历史误差、输出、积分和时间戳清零。 */
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // PID控制器加速度限幅
    , limit(limit)         // PID控制器输出限幅
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();
}

/** @brief 清零 PID 的全部历史状态，避免故障或模式切换后残留积分。 */
void PIDController::reset(){
    error_prev = 0.0f;
    output_prev = 0.0f;
    integral_prev = 0.0f;
    timestamp_prev = micros();
}

/**
 * @brief 计算一次 PID 输出。
 * @param error 本次跟踪误差
 * @return 经过积分/输出限幅和变化率限制的控制量
 */
float PIDController::operator() (float error){
    // 非有限误差会污染全部状态，直接清零并返回安全零输出。
    if (!isfinite(error)) {
        reset();
        return 0.0f;
    }

    // 使用实际循环间隔；过小或超过 0.5 s 的异常间隔按 1 ms 处理。
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // P环
    float proportional = P * error;
    // 梯形积分：使用本次与上次误差的平均值，并限制积分饱和。
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    integral = _constrain(integral, -limit, limit);
    // D环（微分环节）
    float derivative = D*(error - error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

    // ramp > 0 时限制输出变化速率，避免控制量瞬间跳变。
    if(output_ramp > 0){
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    // 保存本次状态，供下一次误差积分、微分和变化率计算使用。
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
