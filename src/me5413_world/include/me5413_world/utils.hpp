#pragma once

#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

double normalizeHeadingError(double error){
    if(error > M_PI){
        error -= 2*M_PI;
    }else if (error < -M_PI){
        error += 2*M_PI;
    }
    return error;
}

double getHeadingError(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
    tf2::Quaternion q_robot;
    tf2::fromMsg(odom_robot.pose.pose.orientation, q_robot);
    const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);

    double roll, pitch, yaw_robot;
    m_robot.getRPY(roll, pitch, yaw_robot);

    tf2::Vector3 point_robot, point_goal;
    tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
    tf2::fromMsg(pose_goal.position, point_goal);
    const tf2::Vector3 delta = point_goal - point_robot;

    double alpha = std::atan2(delta.getY(), delta.getX()) - yaw_robot;
    return normalizeHeadingError(alpha);
}

double getVelFromOdom(const nav_msgs::Odometry& odom_robot){
    tf2::Vector3 robot_vel;
    tf2::fromMsg(odom_robot.twist.twist.linear, robot_vel);
    return robot_vel.length();
}

double getDistance(const tf2::Vector3& a, const tf2::Vector3& b){
    return a.distance(b);
};