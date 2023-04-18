#include "moveit_manager.hpp"



MoveitManager::MoveitManager() : spinner(5)
{

  moveit::planning_interface::MoveGroupInterface::Options opts("manipulator",
                                                               "iiwa/robot_description",
                                                               ros::NodeHandle("iiwa"));
  moveit_group_interface =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(opts);

  //joint_model_group = moveit_group_interface->getCurrentState()->
     // getJointModelGroup("manipulator");

  moveit_group_interface->setMaxVelocityScalingFactor(1);
  moveit_group_interface->setMaxAccelerationScalingFactor(1);

  spinner.start();
}

MoveitManager::~MoveitManager()
{
  spinner.stop();
}


void MoveitManager::planMovementJointPosition(std::array<float, 7> joint_angles){
  std::vector<double> temp(joint_angles.begin(), joint_angles.end());
  moveit_group_interface->setJointValueTarget(temp);
  auto plan = [&](){moveit_group_interface->plan(this->plan);};
  std::thread t = std::thread(plan);
  t.detach();
}

void MoveitManager::moveToPosition(){

  auto move = [&](){moveit_group_interface->asyncMove();};
  std::thread t = std::thread(move);
  t.detach();
}


