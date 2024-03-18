/** path_tracker_node.cpp
 * 
 * Copyright (C) 2024 Shuo SUN & Advanced Robotics Center, National University of Singapore
 * 
 * MIT License
 * 
 * ROS Node for robot to track a given path
 */

#include "me5413_world/path_tracker_node.hpp"

namespace me5413_world 
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double PID_Kp_yaw, PID_Ki_yaw, PID_Kd_yaw;
double STANLEY_K;
bool PARAMS_UPDATED;

double lookahead_dist;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level)
{
  // Common Params
  SPEED_TARGET = config.speed_target;
  // PID 
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;

  lookahead_dist = config.lookahead_dist;

  PID_Kp_yaw = config.PID_Kp_yaw;
  PID_Ki_yaw = config.PID_Ki_yaw;
  PID_Kd_yaw = config.PID_Kd_yaw;
  // Stanley
  STANLEY_K = config.stanley_K;
  
  PARAMS_UPDATED = true;
};

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  this->pub_target_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_point", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";
};

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  tf2::Vector3 pos_robot;
  tf2::fromMsg(this->odom_world_robot_.pose.pose.position, pos_robot);
  
  const auto targetPt = control::PurePursuitController::getTargetPoint(pos_robot, path, lookahead_dist);
  std::cout << "Target pt is " << targetPt << "\n";
  this->pose_world_goal_ = path->poses[targetPt].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  geometry_msgs::PoseStamped pose_stamped_goal;
  pose_stamped_goal.pose = this->pose_world_goal_;
  pose_stamped_goal.header.frame_id = this->world_frame_;

  this->pub_target_point_.publish(pose_stamped_goal);
  return;
};

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
};

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  geometry_msgs::Twist cmd_vel;
  if (PARAMS_UPDATED)
  {
    _pid.updateParams(control::PIDController::Params{PID_Kp, PID_Ki, PID_Kd, PID_Kp_yaw, PID_Ki_yaw, PID_Kd_yaw, SPEED_TARGET});
    PARAMS_UPDATED = false;
  }
  
  cmd_vel.linear.x = _pid.getLinearVel(odom_robot, pose_goal);
  cmd_vel.angular.z = _pid.getPIDSteering(odom_robot, pose_goal);
  // cmd_vel.linear.x = 0.3;
  
  // const auto u = _dwa.getControl(odom_robot, pose_goal);
  // cmd_vel.linear.x = u[0];
  // cmd_vel.angular.z = u[1];

  // cmd_vel.angular.z = _pp.getPurePursuitSteering(odom_robot, pose_goal);
  // cmd_vel.angular.z = _pp.getStanleySteering(odom_robot, pose_goal);
  cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -2.2), 2.2);
  
  std::cout << "cmd vel lin x " << cmd_vel.linear.x << " cmd vel ang z " << cmd_vel.angular.z << "\n";
  return cmd_vel;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}