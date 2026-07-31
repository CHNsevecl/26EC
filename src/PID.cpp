#include "../include/PID.hpp"
#include <iostream>
#include <cmath>
#include <chrono>

// ==================== PIDController 实现 ====================

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double output_min, double output_max)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      integral_(0.0), prev_error_(0.0),
      output_min_(output_min), output_max_(output_max),
      first_run_(true),
      dt_(0.001),  // 默认1ms
      dt_max_(0.1) // 默认最大100ms
{
    last_time_ = std::chrono::steady_clock::now();
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_run_ = true;
    last_time_ = std::chrono::steady_clock::now();
}

void PIDController::setIntegral(double integral) {
    integral_ = integral;
}

double PIDController::update(double error) {
    // 自动计算dt
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_time_;
    
    double dt = elapsed.count();
    
    // 限制dt范围
    if (dt < 0.000001) dt = 0.000001;  // 最小1微秒
    if (dt > dt_max_) dt = dt_max_;    // 防止异常跳变

    
    last_time_ = current_time;
    dt_ = dt;
    
    // 比例项
    double proportional = Kp_ * error;
    
    // 积分项
    integral_ += Ki_ * error * dt;
    
    // 微分项
    double derivative = 0.0;
    if (!first_run_) {
        derivative = Kd_ * (error - prev_error_) / dt;
        
    } else {
        first_run_ = false;
    }
    
    // 计算输出
    double output = proportional + integral_ + derivative;
    
    // 输出限幅
    output = std::max(output_min_, std::min(output_max_, output));
    
    // 保存当前误差供下次微分计算
    prev_error_ = error;
    
    return output;
}

double PIDController::getProportional() const {
    return Kp_ * prev_error_;
}

double PIDController::getIntegral() const {
    return integral_;
}

double PIDController::getDerivative() const {
    return 0.0;  // 简化实现
}

void PIDController::setGains(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void PIDController::setOutputLimits(double min, double max) {
    output_min_ = min;
    output_max_ = max;
}

// ==================== IncrementalPIDController 实现 ====================

IncrementalPIDController::IncrementalPIDController(double Kp, double Ki, double Kd,
                                                   double output_min, double output_max)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      prev_error_(0.0), prev_prev_error_(0.0),
      output_(0.0),
      output_min_(output_min), output_max_(output_max),
      dt_(0.001),
      dt_max_(0.1),
      first_run_(true)
{
    last_time_ = std::chrono::steady_clock::now();
}

void IncrementalPIDController::reset() {
    prev_error_ = 0.0;
    prev_prev_error_ = 0.0;
    output_ = 0.0;
    first_run_ = true;
    last_time_ = std::chrono::steady_clock::now();
}

double IncrementalPIDController::update(double error) {
    // 自动计算dt
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - last_time_;
    double dt = elapsed.count();
    
    // 限制dt范围
    if (dt < 0.000001) dt = 0.000001;
    if (dt > dt_max_) dt = dt_max_;
    
    last_time_ = current_time;
    dt_ = dt;
    
    // 增量式PID: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt
    double delta = 0.0;
    
    if (!first_run_) {
        delta = Kp_ * (error - prev_error_) +
                Ki_ * error * dt +
                Kd_ * (error - 2 * prev_error_ + prev_prev_error_) / dt;
    } else {
        // 第一次运行，只使用比例项
        delta = Kp_ * error;
        first_run_ = false;
    }
    
    // 更新输出
    output_ += delta;
    
    // 输出限幅
    output_ = std::max(output_min_, std::min(output_max_, output_));
    
    // 更新历史误差
    prev_prev_error_ = prev_error_;
    prev_error_ = error;
    
    return delta;  // 返回增量值
}

void IncrementalPIDController::setGains(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

void IncrementalPIDController::setOutputLimits(double min, double max) {
    output_min_ = min;
    output_max_ = max;
}