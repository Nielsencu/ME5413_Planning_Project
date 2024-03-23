#pragma once
#include "types.hpp"

namespace control{
    class BaseController{
    public:
        virtual CmdVel getCmdVel(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal, const nav_msgs::Path::ConstPtr& path) = 0;
    };
}