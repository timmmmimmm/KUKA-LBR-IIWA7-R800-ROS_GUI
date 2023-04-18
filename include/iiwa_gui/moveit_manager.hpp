#pragma once
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/RobotTrajectory.h>



class MoveitManager
{
public:
  MoveitManager();
  ~MoveitManager();
  void planMovementJointPosition(std::array<float,7> joint_angles);
  void moveToPosition();

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveit_group_interface = nullptr;
  const moveit::core::JointModelGroup* joint_model_group;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::AsyncSpinner spinner;
};

