#pragma once

//Qt and C++ stuff
#include <QMainWindow>
#include <QtWidgets>
#include <QTimer>
#include <QProcess>
#include <QCloseEvent>
#include <array>
#include <functional>
#include <future>
#include <sstream>
#include <utility>


//core ros stuff
#include <ros/ros.h>

#include <iiwa_ros/state/joint_torque.hpp>
#include <iiwa_ros/state/joint_velocity.hpp>
#include <iiwa_ros/state/cartesian_wrench.hpp>

//widgets
#include "joint_position_ui.h"
#include "cartesian_position_ui.h"
#include "universal_popup.h"
#include "sim_controller.h"
#include "moveit_manager.hpp"

//subscribers
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/CartesianWrench.h>
#include <iiwa_msgs/SetSmartServoJointSpeedLimits.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

signals:
  void robotStatus(bool isOnline);
  void rvizStatus(bool isOnline);
  void gazeboStatus(bool isOnline);

public slots:
  void spinNode();
  void checkIfOnline();
  void jointPositionUICreated(bool status);
  void cartesianPositionUiCreated(bool status);
  void rvizOperational(bool);
  void gazeboOperational(bool);

private slots:
  void on_comboBox_currentIndexChanged(int index);


  void on_actionCloseRviz_triggered();

  void on_actionLaunchGazebo_triggered();

  void on_actionCloseGazebo_triggered();

  void on_actionRvizWith_Sim_triggered();

  void on_actionRvizNo_Sim_triggered();

  void on_stateSelector_currentIndexChanged(int index);

private:
  Ui::MainWindow *ui; //Qt main ui
  bool shutdownProcedure = false,
  isRobotOnline = false,
  jointPositionUICreated_ = false,
  cartesianPositionUiCreated_ = false,
  rvizOnline = false,
  gazeboOnline = false;

  ros::NodeHandle node_handle_;
  QTimer* spinner;

  //Joint position control and status widget
  JointPositionUI* jointPositionUi;
  CartesianPositionUi* cartesianPositionUi;
  UniversalPopup* popup;
  SimController* simController;
  MoveitManager* moveitManager = nullptr;

  std::stringstream ss_joint_torque_, ss_joint_velocity_, ss_cartesian_force, ss_cartesian_torque;

  //torque values are X10 because of sliders
  std::array<int_fast32_t, 7> joint_torques = {0,0,0,0,0,0,0};
  std::array<QSlider*, 7> sliders_joint_torques = {};
  std::array<QLabel*, 7> labels_joint_torques = {};

  //velocity values are X10 000 (X1000 because of conversion from m/s to mm/s, X10 because of sliders)
  std::array<int_fast32_t, 7> joint_velocities = {0,0,0,0,0,0,0};
  std::array<QSlider*, 7> sliders_joint_velocities = {};
  std::array<QLabel*, 7> labels_joint_velocities = {};

  //{X,XError}, {Y,YError}, {Z,ZError} |||| values of non error components X10 because of sliders
  std::array<std::pair<int_fast32_t, double>, 3> cartesian_torque {std::make_pair(0, 0.0),
                                                                  std::make_pair(0, 0.0),
                                                                  std::make_pair(0, 0.0)};
  std::array<std::pair<int_fast32_t, double>, 3> cartesian_force {std::make_pair(0, 0.0),
                                                                 std::make_pair(0, 0.0),
                                                                 std::make_pair(0, 0.0)};
 //{ValueLabel, InaccuracyLabel}
  std::array<std::pair<QLabel *, QLabel *>, 3> labels_cartesian_torque {};
  std::array<std::pair<QLabel *, QLabel *>, 3> labels_cartesian_force {};

  std::array<QSlider *, 3> sliders_cartesian_torque {};
  std::array<QSlider *, 3> sliders_cartesian_force {};

  //subscribers
  ros::Subscriber subscriber_robot_online;
  iiwa_ros::state::JointTorque subscriber_joint_torque_;
  iiwa_ros::state::JointVelocity subscriber_joint_velocity_;
  iiwa_ros::state::CartesianWrench subscriber_cartesian_wrench_;

  ros::ServiceClient speedControlClient;

  void closeEvent(QCloseEvent* event);

  void createUniversalPopup(std::string&& windowName,
                            std::string&& prompt,
                            std::string&& positiveButtonText,
                            QIcon* icon = nullptr,
                            std::function<bool()> callable = nullptr);

  void createLoadingPopup(std::string&& windowName,
                          std::string&& prompt,
                          std::function<bool()> callable);

  void initializeRobotSpeed();

  //callbacks
  void callbackOnline(const std_msgs::Bool &isOnline);
  void callbackJointTorque(const iiwa_msgs::JointTorque &joint_torque_);
  void callbackJointVelocity(const iiwa_msgs::JointVelocity &joint_velocity_);
  void callbackCartesianWrech(const iiwa_msgs::CartesianWrench &cartesian_wrench_);
};















