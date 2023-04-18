#pragma once

#include <QWidget>
#include <QtWidgets>

//Cpp stuff
#include <string>
#include <functional>
#include <sstream>
#include <iomanip>
#include <array>

#include<angles/angles.h>

//iiwa msgs
#include <iiwa_msgs/JointPosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <std_msgs/Time.h>

//position publisher
#include <iiwa_ros/command/joint_position.hpp>

//subscribers
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/state/joint_torque.hpp>
#include <iiwa_ros/state/joint_velocity.hpp>
#include <iiwa_ros/state/destination_reached.hpp>

//clients
#include <iiwa_ros/service/time_to_destination.hpp>

#include "moveit_manager.hpp"


namespace Ui {
class JointPositionUI;
}

class JointPositionUI : public QWidget
{
  Q_OBJECT

public:
  explicit JointPositionUI(QWidget *parent = nullptr);
  ~JointPositionUI();

signals:
  void uiCameToLife(bool status);

public slots:
  void robotState(bool isOnline);
  void rvizState(bool isOnline);

private slots:


  void on_slider_A1_sliderMoved(int position);

  void on_slider_A2_sliderMoved(int position);

  void on_slider_A3_sliderMoved(int position);

  void on_slider_A4_sliderMoved(int position);

  void on_slider_A5_sliderMoved(int position);

  void on_slider_A6_sliderMoved(int position);

  void on_slider_A7_sliderMoved(int position);

  void on_homePosButton_clicked();

  void on_setPosButton_clicked();


  void on_stateSelector_currentIndexChanged(int index);

  void on_slider_A1_sliderReleased();

  void on_slider_A2_sliderReleased();

  void on_slider_A3_sliderReleased();

  void on_slider_A4_sliderReleased();

  void on_slider_A5_sliderReleased();

  void on_slider_A6_sliderReleased();

  void on_slider_A7_sliderReleased();

private:
  Ui::JointPositionUI *ui;

  //publishers for robot control
  iiwa_ros::command::JointPosition publisher_joint_position_;

  //subscribers for robot states
  iiwa_ros::state::JointPosition subscriber_joint_position_;
  iiwa_ros::state::JointTorque subscriber_joint_torque_;
  iiwa_ros::state::JointVelocity subscriber_joint_velocity_;
  iiwa_ros::state::DestinationReached subscriber_destination_reached_;
  iiwa_ros::service::TimeToDestinationService client_time_to_destination_;

  MoveitManager* moveitManager = nullptr;

  //axis angles
  //float A1 = 0, A2 = 0, A3 = 0, A4 = 0, A5 = 0, A6 = 0, A7 = 0;
  std::array<float, 7> joint_angles = {0,0,0,0,0,0,0};
  std::array<QSlider*, 7> sliders_joint_angles = {};

  //torque values are X10 because of sliders
  std::array<int_fast32_t, 7> joint_torques = {0,0,0,0,0,0,0};
  std::array<QSlider*, 7> sliders_joint_torques = {};
  std::array<QLabel*, 7> labels_joint_torques = {};

  //velocity values are X10 000 (X1000 because of conversion from m/s to mm/s, X10 because of sliders)
  std::array<int_fast32_t, 7> joint_velocities = {0,0,0,0,0,0,0};
  std::array<QSlider*, 7> sliders_joint_velocities = {};
  std::array<QLabel*, 7> labels_joint_velocities = {};

  //flags
  bool gui_init_ = true, destination_reached_ = true, robotOnline = false, rvizOnline = false;

  //tool tip formatters
  std::stringstream ss_tool_tip_, ss_joint_torque_, ss_joint_velocity_;

  //callbacks
  void callbackJointState(const iiwa_msgs::JointPosition &joint_position_);
  void callbackJointTorque(const iiwa_msgs::JointTorque &joint_torque_);
  void callbackJointVelocity(const iiwa_msgs::JointVelocity &joint_velocity_);
  void callbackDestinationReached(const std_msgs::Time &timestamp);

  void setPosition();
  void planPosition();
};

