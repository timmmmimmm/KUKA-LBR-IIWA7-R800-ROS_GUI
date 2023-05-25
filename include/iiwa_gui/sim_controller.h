#pragma once

#include <QObject>

#include <future>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class SimController : public QObject
{
  Q_OBJECT
public:
  explicit SimController(QObject *parent = nullptr);
  bool lauchBoth();
  bool stopSims();
  bool launchRviz(bool withGazebo);
  bool launchGazebo();

signals:
  void rvizStatus(bool online);
  void rvizFailed(std::string message);

  void gazeboStatus(bool online);
  void gazeboFailed(std::string message);

private:
  ros::ServiceClient rviz_client_, gazebo_client_, sim_stop_client_;
  ros::NodeHandle node_handle_;
};

