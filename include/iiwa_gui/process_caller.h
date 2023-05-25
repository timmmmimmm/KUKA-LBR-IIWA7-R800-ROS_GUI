#pragma once

#include <QObject>
#include <QProcess>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class ProcessCaller : public QObject
{
  Q_OBJECT
public:
  explicit ProcessCaller(QObject *parent = nullptr);

signals:

private:
  ros::ServiceServer gazeboLaunchServer, rvizLaunchServer, killSimServer;
  ros::NodeHandle nh;

  QProcess rvizProcess, gazeboProcess;

  bool gazeboLaunched = false, rvizLaunched = false;

  bool killSims(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool launchRviz(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool launchGazebo(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

};

