#ifndef PID_H
#define PID_H

/**
 * @file pid.h
 * @brief 基于实际采样时间的数字 PID，包含积分限幅、输出限幅和变化率限制。
 */
class PIDController
{
public:
    /**
     * @brief 初始化 PID 参数和内部状态。
     * @param P 比例增益
     * @param I 积分增益
     * @param D 微分增益
     * @param ramp 输出最大变化率；<=0 表示不启用变化率限制
     * @param limit 积分项和总输出的绝对值限幅
     */
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    /**
     * @brief 根据误差计算一次 PID 输出。
     * @param error 本次目标值与反馈值之差
     * @return 经过限幅和变化率限制后的控制量
     */
    float operator() (float error);

    /** @brief 清零误差历史、积分、输出和时间戳，用于故障恢复或模式切换。 */
    void reset();

    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; //!< 输出最大变化率，单位控制量/秒
    float limit; //!< 积分项和总输出限幅
protected:
    float error_prev; //!< 上一次误差，用于积分和微分
    float output_prev;  //!< 上一次输出，用于变化率限制
    float integral_prev; //!< 上一次积分分量，用于梯形积分
    unsigned long timestamp_prev; //!< 上次执行时间戳，单位 us
};

#endif
