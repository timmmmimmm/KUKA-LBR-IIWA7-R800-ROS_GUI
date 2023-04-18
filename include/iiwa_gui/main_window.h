#pragma once

//Qt and C++ stuff
#include <QMainWindow>
#include <QtWidgets>
#include <QTimer>
#include <QProcess>
#include <functional>
#include <future>


//core ros stuff
#include <ros/ros.h>

//widgets
#include "joint_position_ui.h"
#include "universal_popup.h"

//subscribers
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

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
  void checkIfOnline();
  void spinNode();

private slots:
  void on_comboBox_currentIndexChanged(int index);
  void uiIsAlive(bool status);

  void on_actionLaunch_triggered();

private:
  Ui::MainWindow *ui; //Qt main ui
  bool isRobotOnline = false;
  bool isUIOnline = false;
  bool rvizOnline = false;
  bool gazeboOnline = false;

  ros::NodeHandle node_handle_;
  QTimer* spinner;

  //Joint position control and status widget
  JointPositionUI* jointPositionUi;
  UniversalPopup* popup;

  //subscribers
  ros::Subscriber subscriber_robot_online;

  bool launchSim(bool withGazebo);

  //callbacks
  void callbackOnline(const std_msgs::Bool &isOnline);
};















