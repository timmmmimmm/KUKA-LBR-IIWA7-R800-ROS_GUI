#pragma once

#include <QWidget>
#include <QDoubleSpinBox>
#include <functional>
#include <array>


#include <ros/ros.h>
#include <ros/console.h>

#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/cartesian_pose_linear.hpp>

#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/destination_reached.hpp>

#include <iiwa_msgs/CartesianPose.h>
#include <std_msgs/Time.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include "moveit_manager.hpp"

namespace Ui {
class CartesianPositionUi;
}

class CartesianPositionUi : public QWidget
{
  Q_OBJECT

public:
  explicit CartesianPositionUi(QWidget *parent = nullptr);
  ~CartesianPositionUi();
  void setMoveitManager(MoveitManager* moveitManager);
  void setFocusedWidget(bool focus);

signals:
  void cartesianPositionUIExists(bool status);
  void setHome();

public slots:
  void robotState(bool isOnline);
  void rvizState(bool isOnline);

private slots:

  void on_homePosButton_clicked();

  void on_setPosButton_clicked();

  void on_spinBoxPositionX_editingFinished();

  void on_spinBoxPositionY_editingFinished();

  void on_spinBoxPositionZ_editingFinished();

  void on_spinBoxOrientationA_editingFinished();

  void on_spinBoxOrientationB_editingFinished();

  void on_spinBoxOrientationC_editingFinished();

  void on_spinBoxPositionX_valueChanged(double arg1);

  void on_spinBoxPositionY_valueChanged(double arg1);

  void on_spinBoxPositionZ_valueChanged(double arg1);

  void on_spinBoxOrientationA_valueChanged(double arg1);

  void on_spinBoxOrientationB_valueChanged(double arg1);

  void on_spinBoxOrientationC_valueChanged(double arg1);

private:
  Ui::CartesianPositionUi *ui;

  MoveitManager* moveitManager = nullptr;
  ros::NodeHandle node_handle_;


  iiwa_ros::command::CartesianPose publisher_cartesian_pose_;
  iiwa_ros::command::CartesianPoseLinear publisher_cartesian_pose_linear_;

  iiwa_ros::state::CartesianPose subscriber_cartesian_pose_;
  iiwa_ros::state::DestinationReached subscriber_destination_reached_;
  ros::Subscriber subscriber_cartesian_pose_angles_;

  //{X,Y,Z} in milimeters
  std::array<double, 3> positions {0,0,0};
  std::array<QDoubleSpinBox *, 3> positionSpinBoxes {};

  //{A,B,C} in degrees
  std::array<double, 3> orientations{0,0,0};
  std::array<QDoubleSpinBox *, 3> orientationSpinBoxes {};

  bool hasFocus = false,
  robotOnline = false,
  rvizOnline = false,
  initPos = true,
  initRot = true,
  destinationReached = true;

  void planPosition();
  void updateGUIDisplayPose(std::array<double,3> position, std::array<double, 3> orientation);

  void callbackCartesianPose(const iiwa_msgs::CartesianPose &pose);
  void callbackDestinationReached(const std_msgs::Time &timeStamp);
  void callbackCartesianPoseAngles(const std_msgs::String &abcString);
};


