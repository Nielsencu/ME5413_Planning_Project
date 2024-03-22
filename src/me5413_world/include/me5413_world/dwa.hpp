#pragma once
#include <iostream>
#include <array>
#include <vector>
#include "base_controller.hpp"
#include "utils.hpp"

namespace control{
    class DWAController : public BaseController{
    public:
        DWAController() = default;
        struct DWAParams{
            double dt = 0.1;
            double pred_time = 1.0; 
            double max_lin_vel = 0.5; 
            double min_lin_vel = 0.0;
            double max_ang_vel = 3.14;
            double max_lin_acc = 0.5;
            double max_ang_acc = 0.7; 
            double lin_vel_res = 0.01;
            double ang_vel_res = 0.02;
            double speed_cost_gain = 1.0; 
            double angle2goal_cost_gain = 0.15;
            double dist2goal_cost_gain = 0.5;
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
            
            const State x_init{point_robot.getX(), point_robot.getY(), yaw_robot, odom_robot.twist.twist.linear.x, odom_robot.twist.twist.angular.z};
            const Point goal{point_goal.getX(), point_goal.getY()};

            Velocity cmd_vel;
            const auto dw = _getDynamicWindow(x_init);
            _getDWAPlan(x_init, dw , goal, cmd_vel);
            return {cmd_vel.lin_x, cmd_vel.ang_z};
        }

    private:
        DWAParams _params;
        struct Velocity{
            double lin_x;
            double ang_z;
        };

        struct Point{
            double x;
            double y;
        };
        struct State{
            State(double xIn, double yIn, double yawIn, double linXIn, double angZIn):
            point{xIn, yIn}, yaw(yawIn), vel{linXIn, angZIn}{};
            Point point;
            double yaw;
            Velocity vel;
        };

        struct DynamicWindow{
            double vel_min;
            double vel_max;
            double yaw_rate_min;
            double yaw_rate_max;
            std::vector<double> possibleV;
            std::vector<double> possibleW;
        };

        DynamicWindow _getDynamicWindow(const State& x){
            double minV = std::max(_params.min_lin_vel, x.vel.lin_x - _params.max_lin_acc * _params.dt);
            double maxV = std::min(_params.max_lin_vel, x.vel.lin_x + _params.max_lin_acc * _params.dt);
            double minW = std::max(-_params.max_ang_vel, x.vel.ang_z - _params.max_ang_acc * _params.dt);
            double maxW = std::min(_params.max_ang_vel, x.vel.ang_z + _params.max_ang_acc * _params.dt);

            size_t nPossibleV = (maxV - minV) / _params.lin_vel_res;
            size_t nPossibleW = (maxW - minW) / _params.ang_vel_res;

            if(minV > maxV){
                nPossibleV = 0;
            }
            if(minW > maxW){
                nPossibleW = 0;
            }

            std::cout << "nPossibleV " << nPossibleV << "\n";
            std::cout << "nPossibleW " << nPossibleW << "\n";

            std::cout << "maxV " << maxV << " minV " << minV << "\n";
            std::cout << "maxW " << maxW << " minW " << minW << "\n";

            std::vector<double> possibleV;
            std::vector<double> possibleW;

            for(size_t i=0; i < nPossibleV; i++) {
                possibleV.push_back(minV + static_cast<double>(i) * _params.lin_vel_res);
            }

            for(size_t i=0; i < nPossibleW; i++) {
                possibleW.push_back(minW + static_cast<double>(i) * _params.ang_vel_res);
            }

            return {minV, maxV, minW, maxW, possibleV, possibleW};
        }

        void _motion(State& x, const Velocity& u){
            x.yaw += u.ang_z * _params.dt;
            x.point.x += u.lin_x * std::cos(x.yaw) * _params.dt;
            x.point.y += u.lin_x * std::sin(x.yaw) * _params.dt;
            x.vel.lin_x = u.lin_x;
            x.vel.ang_z = u.ang_z;
        }

        void _predictTrajectory(const State& x_init, Velocity cmd_vel, std::vector<Point>& traj){
            double time{0.0};
            State x = x_init;
            traj.push_back({x.point.x, x.point.y});
            while(time <= _params.pred_time){
                _motion(x, cmd_vel);
                traj.push_back({x.point.x, x.point.y});
                time += _params.dt;
            }
        }

        double _getAngle2GoalCost(const std::vector<Point>& traj, const Point& goal, double robot_yaw){
            const Point& traj_end{traj.back()};
            double dx{goal.x - traj_end.x};
            double dy{goal.y - traj_end.y};
            const auto error_angle{std::atan2(dy, dx)};
            const auto cost_angle{normalizeHeadingError(error_angle - robot_yaw)};
            return std::abs(std::atan2(std::sin(cost_angle), std::cos(cost_angle)));
        }

        double _getDist2GoalCost(const std::vector<Point>& traj, const Point& goal){
            const auto traj_end{traj.back()};
            double dx{goal.x - traj_end.x};
            double dy{goal.y - traj_end.y};
            return std::sqrt(dx*dx + dy*dy);
        }

        double _getVelocityCost(const Velocity& vel){
            return _params.max_lin_vel - vel.lin_x;
        }

        void _getDWAPlan(const State& x, const DynamicWindow& dw, const Point& goal, Velocity& velOut){
            double min_cost{10000};
            State x_init = x;
            double cost;
            Velocity vel;
            for(size_t i=0;i<dw.possibleV.size();i++){
                for(size_t j=0;j<dw.possibleW.size();j++){
                    vel.lin_x = dw.possibleV[i];
                    vel.ang_z = dw.possibleW[j];
                    std::vector<Point> temp_traj;
                    _predictTrajectory(x_init, vel, temp_traj);
                    // for(const auto& point : temp_traj){
                    //   ROS_INFO("%f, %f", point.x, point.y);
                    // }
                    // ROS_INFO("------------------------------");
                    ROS_INFO("%f %f %f", _getDist2GoalCost(temp_traj, goal), _getAngle2GoalCost(temp_traj, goal, x_init.yaw), _getVelocityCost(vel));
                    cost = _params.dist2goal_cost_gain * _getDist2GoalCost(temp_traj, goal) \
                            + _params.angle2goal_cost_gain * _getAngle2GoalCost(temp_traj, goal, x_init.yaw) \
                            + _params.speed_cost_gain * _getVelocityCost(vel);

                    if(cost < min_cost){
                        min_cost = cost;
                        velOut = vel;
                    }
                }
            }
        }
    };
}