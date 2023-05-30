#include "moveit_manager.hpp"



MoveitManager::MoveitManager(bool gazeboOnline)
{
  if(!gazeboOnline){
    ros::NodeHandle nh("/iiwa");

    nh.setCallbackQueue(&callback_queue_);

    spinner = std::make_shared<ros::AsyncSpinner>(5, &callback_queue_);

    spinner->start();

    moveit::planning_interface::MoveGroupInterface::Options opts("manipulator",
                                                                 "iiwa/robot_description",
                                                                 nh);
    moveit_group_interface =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(opts);

    joint_model_group = moveit_group_interface->getRobotModel()->getJointModelGroup("manipulator");
    state = moveit_group_interface->getCurrentState();

    moveit_group_interface->setMaxVelocityScalingFactor(1);
    moveit_group_interface->setMaxAccelerationScalingFactor(1);
    moveit_group_interface->setGoalTolerance(0.001);
  }
}

MoveitManager::~MoveitManager()
{
  moveit_group_interface->stop();
  moveit_group_interface->clearPoseTargets();
  callback_queue_.clear();
  spinner->stop();
}


void MoveitManager::planMovementJointPosition(std::array<float, 7> joint_angles){

  if(spinner->canStart())
    spinner->start();

  moveit_group_interface->clearPoseTargets();

  std::vector<double> temp(joint_angles.begin(), joint_angles.end());

  for(auto & angle : temp){
    angle = angles::from_degrees(angle);
  }

  moveit_group_interface->setJointValueTarget(temp);
  auto plan = [&](){moveit_group_interface->plan(this->plan);};

  std::thread t = std::thread(plan);
  t.detach();
}

void MoveitManager::moveToPosition(){
  if(spinner->canStart())
    spinner->start();

  auto move = [&](){moveit_group_interface->move();
                    moveit_group_interface->setStartStateToCurrentState();
                    spinner->stop();};

  std::thread t = std::thread(move);
  t.detach();
}

void MoveitManager::setStartPosition(std::array<float, 7> joint_angles)
{
  planMovementJointPosition(joint_angles);

  moveToPosition();
}

void MoveitManager::planCarteianPose(std::array<double,3> position, std::array<double,3> orientation)
{

  for(int_fast8_t i = 0; i < 3; i++)
    orientation.at(i) = angles::from_degrees(orientation.at(i));

  if(spinner->canStart())
    spinner->start();

  moveit_group_interface->clearPoseTargets();

  geometry_msgs::Pose pose;

  pose.position.x = position.at(0);
  pose.position.y = position.at(1);
  pose.position.z = position.at(2);

  tf::Quaternion quat;
  quat.setRPY(orientation.at(2), orientation.at(1), orientation.at(0));

  pose.orientation.x = quat.x();

  pose.orientation.y = quat.y();

  pose.orientation.z = quat.z();

  pose.orientation.w = quat.w();

  moveit_group_interface->setPoseTarget(pose);

  auto plan = [&](){moveit_group_interface->plan(this->plan);};
  std::thread t = std::thread(plan);
  t.detach();
}

std::shared_ptr<std::array<std::pair<double, double>, 3>> MoveitManager::getCurrentPose()
{
  if(spinner->canStart())
    spinner->start();

  state = moveit_group_interface->getCurrentState();

  auto abcPose = std::make_shared<std::array<std::pair<double,double>, 3>>();
  const Eigen::Affine3d& eeState = state->getGlobalLinkTransform("iiwa_link_ee");
  Eigen::Vector3d position = eeState.translation();
  Eigen::Matrix3d orientation = eeState.rotation();

  tf::Matrix3x3 tfOrientation;
  tf::matrixEigenToTF(orientation,tfOrientation);

  tf::Quaternion orientationQuat;

  tfOrientation.getRotation(orientationQuat);

  std::array<double, 3> abc {std::atan2(2*(orientationQuat.w() * orientationQuat.z() + orientationQuat.x() * orientationQuat.y()),
                                        1 - 2*(std::pow(orientationQuat.y(), 2) + std::pow(orientationQuat.z(),2))),
                             std::asin(2*(orientationQuat.w() * orientationQuat.y() - orientationQuat.z() * orientationQuat.x())),
                             std::atan2(2*(orientationQuat.w() * orientationQuat.x() + orientationQuat.y() * orientationQuat.z()),
                                        1 - 2*(std::pow(orientationQuat.x(), 2) + std::pow(orientationQuat.y(),2)))};

  for(int_fast8_t i = 0; i < 3; ++i){
    abcPose->at(i).first = position[i];

    abcPose->at(i).second = angles::to_degrees(abc[i]);
  }

  spinner->stop();
  return abcPose;
}



