#pragma once
#include <thread>
#include <vector>
#include <array>
#include <utility>

#include <ros/console.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>

#include <angles/angles.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


class MoveitManager
{
public:
  MoveitManager(bool gazeboOnline);
  ~MoveitManager();
  void planMovementJointPosition(std::array<float,7> joint_angles);
  void moveToPosition();
  void setStartPosition(std::array<float,7> joint_angles);
  void planCarteianPose(std::array<double,3> position, std::array<double,3> orientation);
  std::shared_ptr<std::array<std::pair<double, double>, 3>> getCurrentPose();

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveit_group_interface = nullptr;
  const moveit::core::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::shared_ptr<ros::AsyncSpinner> spinner;
  ros::CallbackQueue callback_queue_;
  moveit::core::RobotStatePtr state;
};

