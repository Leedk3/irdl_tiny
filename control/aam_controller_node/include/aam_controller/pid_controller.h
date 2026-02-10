#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#pragma once

#include <string>
#include <algorithm>

class PIDController
{
public:
  PIDController(double kp, double ki, double kd, double max_out, double min_out, const std::string &name = "pid")
  : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), min_out_(min_out), name_(name)
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

  // 주기적으로 호출 (error: 현재 오차, dt: 시간 [초])
  double update(double error, double dt)
  {
    // 적분항
    integral_ += error * dt;
    if(integral_ > max_out_ * 2.0)
        integral_ = max_out_ * 2.0;
    else if(integral_ < min_out_ * 2.0)
        integral_ = min_out_ * 2.0;

    // 미분항
    double derivative = 0.0;
    if(dt > 1e-6)
      derivative = (error - prev_error_) / dt;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 출력 제한
    if(output > max_out_)
      output = max_out_;
    else if(output < min_out_)
      output = min_out_;

    // 이전 값 갱신
    prev_error_ = error;

    return output;
  }

  void reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

private:
  double kp_;
  double ki_;
  double kd_;
  double max_out_;
  double min_out_;

  double prev_error_;
  double integral_;

  std::string name_;
};

class LowPassFilter
{
public:
  // alpha in (0, 1), initial_value는 초기 출력값
  LowPassFilter(double alpha, double initial_value = 0.0)
    : alpha_(alpha), y_prev_(initial_value)
  {}

  // 새로운 값 x를 필터링해서 반환
  double filter(double x)
  {
    double y = (1.0 - alpha_) * y_prev_ + alpha_ * x;
    y_prev_ = y;
    return y;
  }

  // 필터 상태(이전값) 리셋
  void reset(double value)
  {
    y_prev_ = value;
  }

private:
  double alpha_;
  double y_prev_;
};


#endif //PID_CONTROLLER_H