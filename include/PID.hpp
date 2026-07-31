#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>
#include <chrono>

// 位置式PID控制器（自动计算dt）
class PIDController {
public:
    // 构造函数
    PIDController(double Kp = 1.0, double Ki = 0.0, double Kd = 0.0,
                  double output_min = -INFINITY, double output_max = INFINITY);
    
    // 重置积分项和上次误差
    void reset();
    
    // 设置积分项（用于积分限幅或外部干预）
    void setIntegral(double integral);
    
    // 更新PID输出（输入为误差值，自动计算dt）
    double update(double error);
    
    // 获取当前各项参数（用于调试）
    double getProportional() const;
    double getIntegral() const;
    double getDerivative() const;
    double getPreviousError() const { return prev_error_; }
    double getDt() const { return dt_; }
    
    // 设置参数
    void setGains(double Kp, double Ki, double Kd);
    void setOutputLimits(double min, double max);
    void setDtMax(double dt_max) { dt_max_ = dt_max; }  // 设置最大dt限制
    
    // 获取参数
    double getKp() const { return Kp_; }
    double getKi() const { return Ki_; }
    double getKd() const { return Kd_; }
    double getOutputMin() const { return output_min_; }
    double getOutputMax() const { return output_max_; }

private:
    double Kp_;          // 比例系数
    double Ki_;          // 积分系数
    double Kd_;          // 微分系数
    
    double integral_;    // 积分累积
    double prev_error_;  // 上一次误差
    
    double output_min_;  // 输出下限
    double output_max_;  // 输出上限
    
    bool first_run_;     // 首次运行标志
    
    // 自动计算dt相关
    std::chrono::steady_clock::time_point last_time_;
    double dt_;          // 上次计算的时间步长
    double dt_max_;      // 最大允许dt（防止异常跳变）
};

// 增量式PID控制器（自动计算dt）
class IncrementalPIDController {
public:
    // 构造函数
    IncrementalPIDController(double Kp = 1.0, double Ki = 0.0, double Kd = 0.0,
                             double output_min = -INFINITY, double output_max = INFINITY);
    
    // 重置
    void reset();
    
    // 更新（输入为误差值，自动计算dt）
    double update(double error);
    
    // 获取当前输出
    double getOutput() const { return output_; }
    double getDt() const { return dt_; }
    
    // 设置参数
    void setGains(double Kp, double Ki, double Kd);
    void setOutputLimits(double min, double max);
    void setDtMax(double dt_max) { dt_max_ = dt_max; }
    
    // 获取参数
    double getKp() const { return Kp_; }
    double getKi() const { return Ki_; }
    double getKd() const { return Kd_; }
    double getOutputMin() const { return output_min_; }
    double getOutputMax() const { return output_max_; }
    double getPreviousError() const { return prev_error_; }
    double getPrevPrevError() const { return prev_prev_error_; }

private:
    double Kp_;              // 比例系数
    double Ki_;              // 积分系数
    double Kd_;              // 微分系数
    
    double prev_error_;      // e(k-1)
    double prev_prev_error_; // e(k-2)
    double output_;          // 当前输出值
    
    double output_min_;      // 输出下限
    double output_max_;      // 输出上限
    
    // 自动计算dt相关
    std::chrono::steady_clock::time_point last_time_;
    double dt_;              // 上次计算的时间步长
    double dt_max_;          // 最大允许dt
    bool first_run_;         // 首次运行标志
};

#endif // PID_CONTROLLER_HPP