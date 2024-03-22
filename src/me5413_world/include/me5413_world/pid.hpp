/** pid.hpp
 * 
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * MIT License
 * 
 * Implementation of PID controller
 */

#pragma once

#include <iostream>
#include <cmath>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "utils.hpp"
#include "base_controller.hpp"

namespace control
{
class PID
{
 public:
  PID() {};
  PID(double dt, double max, double min, double Kp, double Kd, double Ki);
  ~PID() {};

  void updateSettings(const double Kp, const double Kd, const double Ki);
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(double error);

 private:
  double dt_;
  double max_;
  double min_;
  double Kp_;
  double Kd_;
  double Ki_;
  double pre_error_;
  double integral_;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) :
  dt_(dt),
  max_(max),
  min_(min),
  Kp_(Kp),
  Kd_(Kd),
  Ki_(Ki),
  pre_error_(0),
  integral_(0) 
{};

void PID::updateSettings(const double Kp, const double Kd, const double Ki)
{
  this->Kp_ = Kp;
  this->Kd_ = Kd;
  this->Ki_ = Ki;
};

double PID::calculate(double error)
{
  // Proportional term
  const double P_term = Kp_ * error;

  // Integral term
  integral_ += error * dt_;
  const double I_term = Ki_ * integral_;

  // Derivative term
  const double derivative = (error - pre_error_) / dt_;
  const double D_term = Kd_ * derivative;

  // Calculate total output
  double output = P_term + I_term + D_term;

  // Restrict to max/min
  output = std::min(output, max_);
  output = std::max(output, min_);

  // Save error to previous error
  pre_error_ = error;

  return output;
};

class PIDController : public BaseController{
  public:
    struct Params{
      double Kp = 0.5;
      double Ki = 0.2;
      double Kd = 0.2;
      double Kp_yaw = 1.5;
      double Ki_yaw = 0.2; 
      double Kd_yaw = 0.0;
      double linear_speed_target = 0.5;
    };

    PIDController() : 
    _pid(0.1, 1.0, -1.0, _params.Kp, _params.Ki, _params.Kd),
    _pidYaw(0.1, 2.2, -2.2, _params.Kp_yaw, _params.Ki_yaw, _params.Kd_yaw){};

    void updateParams(Params&& paramsIn){
      _params = std::move(paramsIn);
      _pid.updateSettings(_params.Kp, _params.Ki, _params.Kd);
      _pidYaw.updateSettings(_params.Kp_yaw, _params.Ki_yaw, _params.Kd_yaw);
    }
    
    double getLinX(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
      const auto velocity = getVelFromOdom(odom_robot);
      const auto lin_error = _params.linear_speed_target - velocity;
      return _pid.calculate(lin_error);;
    }

    double getAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
      const auto heading_error = getHeadingError(odom_robot, pose_goal);
      if(std::isnan(heading_error)){
          return 0.0;
      }
      return _pidYaw.calculate(heading_error);
    }
  private:
    Params _params;
    PID _pid;
    PID _pidYaw;
};
} // namespace control
