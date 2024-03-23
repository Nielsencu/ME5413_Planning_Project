#pragma once

#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Path.h>
#include "utils.hpp"
#include "base_controller.hpp"
#include "pid.hpp"

namespace control{

    class StanleyController : public BaseController{
        public:
        struct Params{
            double stanley_k = 0.5;
        };

        PIDController* getPIDController(){return &_pid;};

        void updateParams(Params&& paramsIn){
            _params = std::move(paramsIn);
        };

        double getLinX(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
            return _pid.getLinX(odom_robot, pose_goal);
        }

        double getAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) const{
            // Heading Error
            tf2::Quaternion q_robot, q_goal;
            tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
            tf2::fromMsg(pose_goal.orientation, q_goal);
            const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
            const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

            double roll, pitch, yaw_robot, yaw_goal;
            m_robot.getRPY(roll, pitch, yaw_robot);
            m_goal.getRPY(roll, pitch, yaw_goal);

            const double heading_error = normalizeHeadingError(yaw_robot - yaw_goal);

            // Lateral Error
            tf2::Vector3 point_robot, point_goal;
            tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
            tf2::fromMsg(pose_goal.position, point_goal);
            const tf2::Vector3 V_goal_robot = point_robot - point_goal;
            const double angle_goal_robot = std::atan2(V_goal_robot.getY(), V_goal_robot.getX());
            const double angle_diff = angle_goal_robot - yaw_goal;
            const double lat_error = V_goal_robot.length()*std::sin(angle_diff);

            // Velocity
            const auto velocity = getVelFromOdom(odom_robot);
            const double stanley_output = -1.0*(heading_error + std::atan2(_params.stanley_k*lat_error, std::max(velocity, 0.3)));
            return stanley_output;
        }

        CmdVel getCmdVel(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal, const nav_msgs::Path::ConstPtr& path){
            return {getLinX(odom_robot, pose_goal), getAngZ(odom_robot, pose_goal)};
        }

        private:
            Params _params;
            PIDController _pid;
    };

    class PurePursuitController : public BaseController{
    public:
        struct Params{
            double lookahead_dist = 1.0;
            double kp = 1.0;
        };

        PIDController* getPIDController(){return &_pid;};

        void updateParams(Params&& paramsIn){
            _params = std::move(paramsIn);
        };

        double getLinX(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
            return _pid.getLinX(odom_robot, pose_goal);
        }

        double getAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) const{
            const auto heading_error = getHeadingError(odom_robot, pose_goal);
            const auto velocity = getVelFromOdom(odom_robot);
            return std::atan2(2 * VEHICLE_LENGTH * std::sin(heading_error),   (_params.kp * velocity));
        }

        static int getTargetPoint(const tf2::Vector3& pos, const nav_msgs::Path::ConstPtr& path, double lookahead_dist){
            tf2::Vector3 targetPt;
            int target_idx = getClosestWaypoint(pos, path);
            tf2::fromMsg(path->poses[target_idx].pose.position, targetPt);
            auto curr_dist = getDistance(pos, targetPt);
            while(curr_dist < lookahead_dist and target_idx < path->poses.size()-1){
                target_idx++;
                tf2::fromMsg(path->poses[target_idx].pose.position, targetPt);
                curr_dist = getDistance(pos, targetPt);
            }
            return target_idx;
        }

        static int getClosestWaypoint(const tf2::Vector3& pos, const nav_msgs::Path::ConstPtr& path){
            double min_dist = DBL_MAX;
            int id_closest = 0;
            for (int i = 0; i < path->poses.size(); i++){
                const double dist = std::hypot(pos.getX() - path->poses[i].pose.position.x, pos.getY() - path->poses[i].pose.position.y);
                if (dist <= min_dist){
                    min_dist = dist;
                    id_closest = i;
                }else{
                    break;
                }
            }
            return id_closest;
        }

        CmdVel getCmdVel(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal, const nav_msgs::Path::ConstPtr& path){
            return {getLinX(odom_robot, pose_goal), getAngZ(odom_robot, pose_goal)};
        }
        
    private:
        Params _params;
        PIDController _pid;
        static constexpr double VEHICLE_LENGTH = 0.5;
    };
}