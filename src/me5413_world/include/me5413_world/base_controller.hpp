#pragma once

namespace control{
    class BaseController{
    public:
        virtual double getLinX(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) = 0;
        virtual double getAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) = 0;
    };
}