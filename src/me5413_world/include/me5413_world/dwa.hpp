#pragma once
#include <iostream>
#include <array>
#include <vector>
#include "base_controller.hpp"

namespace control{

    class DWAController : public BaseController{
    public:
        DWAController() = default;

        struct DWAParams{
            double dt = 0.1;
            double pred_time = 0.5; 
            double max_lin_vel = 0.5; 
            double min_lin_vel = 0.0;
            double max_ang_vel = 3.14;
            double max_lin_acc = 0.5;
            double max_ang_acc = 0.7; 
            double lin_vel_res = 0.02;
            double ang_vel_res = 0.04;
            double speed_cost_gain = 0.5; 
            double angle2goal_cost_gain = 0.15;
            double dist2goal_cost_gain = 1.0;
        };

        double getLinX(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
            return 0.0;
        }

        double getAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
            return 0.0;
        }

        std::array<double,2> getControl(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
            tf2::Quaternion q_robot;
            tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
            const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);

            double roll, pitch, yaw_robot;
            m_robot.getRPY(roll, pitch, yaw_robot);

            tf2::Vector3 point_robot, point_goal;
            tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
            tf2::fromMsg(pose_goal.position, point_goal);
            
            const double cur_x = point_robot.getX();
            const double cur_y = point_robot.getY();

            const std::array<double,5> x_init{cur_x, cur_y, yaw_robot, odom_robot.twist.twist.linear.x, odom_robot.twist.twist.angular.z};
            const std::array<double,2> goal{point_goal.getX(), point_goal.getY()};

            std::array<double,2> u;
            std::array<double,4> dw{0.f, 0.f, 0.f, 0.f};
            _getDynamicWindow(x_init, dw);
            _getDWAPlan(x_init, dw, goal, u);
            return u;
        }

    private:
        DWAParams _params;
        void _getDynamicWindow(const std::array<double,5>& x, std::array<double,4>& dw){
            std::array<double,4> v_s{_params.min_lin_vel, _params.max_lin_vel, -_params.max_ang_vel, _params.max_ang_vel};
            std::array<double,4> v_d{x[3] - _params.max_lin_acc * _params.dt, 
                x[3] + _params.max_lin_acc * _params.dt, 
                x[4] - _params.max_ang_acc * _params.dt, 
                x[4] + _params.max_ang_acc * _params.dt};
            // get v_min, v_max, yaw_rate_min, yaw_rate_max
            dw[0] = std::max(v_s[0], v_d[0]);
            dw[1] = std::min(v_s[1], v_d[1]);
            dw[2] = std::max(v_s[2], v_d[2]);
            dw[3] = std::min(v_s[3], v_d[3]);
        }

        void _motion(std::array<double,5>& x, const std::array<double,2>& u, double dt){
            x[2] += u[1] * dt;
            x[0] += u[0] * std::cos(x[2]) * dt;
            x[1] += u[0] * std::sin(x[2]) * dt;
            x[3] = u[0];
            x[4] = u[1];
        }

        void _predictTrajectory(const std::array<double,5>& x_init, double v, double y, double dt, std::vector<std::array<double,2>>& traj){
            auto x = x_init;
            double time{0.0};
            std::array<double,2> tmp{x[0], x[1]};
            traj.push_back(tmp);
            while(time <= _params.pred_time){
                std::array<double,2> u{v,y};
                _motion(x, u, dt);
                tmp[0] = x[0];
                tmp[1] = x[1];
                traj.push_back(tmp);
                time += dt;
            }
        }

        double _getAngle2GoalCost(const std::vector<std::array<double,2>>& traj, const std::array<double,2>& goal){
            const auto traj_end{traj.back()};
            double dx{goal[0] - traj_end[0]};
            double dy{goal[1] - traj_end[1]};
            const auto error_angle{std::atan2(dy, dx)};
            const auto cost_angle{error_angle - traj_end[2]};
            return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
        }

        double _getDist2GoalCost(const std::vector<std::array<double,2>>& traj, const std::array<double,2>& goal){
            const auto traj_end{traj.back()};
            double dx{goal[0] - traj_end[0]};
            double dy{goal[1] - traj_end[1]};
            return std::sqrt(dx*dx + dy*dy);
        }

        void _getDWAPlan(const std::array<double,5>& x, const std::array<double,4>& dw, const std::array<double,2>& goal, std::array<double,2>& u){
            double min_cost{10000};
            auto x_init = x;
            for(double v=dw[0];v<=dw[1];v+=_params.lin_vel_res){
                for(double y=dw[2];y<=dw[3];y+=_params.ang_vel_res){
                std::vector<std::array<double,2>> temp_traj;
                _predictTrajectory(x_init, v, y, _params.dt, temp_traj);
                // for(const auto& point : temp_traj){
                //   ROS_INFO("%f, %f", point[0], point[1]);
                // }
                // ROS_INFO("------------------------------");
                double angle_to_goal_cost{_getAngle2GoalCost(temp_traj, goal)};
                double dist_to_goal_cost{_getDist2GoalCost(temp_traj, goal)};
                double speed_cost{_params.max_lin_vel - v};
                double final_cost = _params.dist2goal_cost_gain * dist_to_goal_cost + _params.angle2goal_cost_gain * angle_to_goal_cost \
                                    + _params.speed_cost_gain * speed_cost;
                if(final_cost < min_cost){
                    min_cost = final_cost;
                    u[0] = v;
                    u[1] = y;
                }
                }
            }
        }
    };
}