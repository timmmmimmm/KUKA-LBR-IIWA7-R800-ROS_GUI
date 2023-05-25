#include "process_caller.h"

ProcessCaller::ProcessCaller(QObject *parent)
  : QObject{parent}
{
  gazeboLaunchServer = nh.advertiseService("launch_gazebo",&ProcessCaller::launchGazebo,this);
  rvizLaunchServer = nh.advertiseService("launch_rviz",&ProcessCaller::launchRviz,this);
  killSimServer = nh.advertiseService("kill_sims", &ProcessCaller::killSims,this);
}


bool ProcessCaller::killSims(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  rvizProcess.terminate();
  gazeboProcess.terminate();

  rvizProcess.waitForFinished();
  rvizLaunched = false;

  gazeboProcess.waitForFinished();
  gazeboLaunched = false;

  resp.success = true;

  return true;
}

bool ProcessCaller::launchRviz(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){

  if(!rvizLaunched){
    if(req.data)
     rvizProcess.start("python3 /opt/ros/noetic/bin/roslaunch iiwa_moveit demo2.launch");
    else
      rvizProcess.start("python3 /opt/ros/noetic/bin/roslaunch iiwa_moveit demo.launch");

    if(rvizProcess.waitForStarted()){
      resp.success = true;
      resp.message = "OK";
      rvizLaunched = true;
    }
    else
    {
      resp.success = false;
      resp.message = "FAILED TO LAUNCH RVIZ";
    }
  }
  else{
    resp.success = false;
    resp.message = "RVIZ ALREADY ONLINE";
  }

  return true;
}

bool ProcessCaller::launchGazebo(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){

  if(!gazeboLaunched){
    gazeboProcess.start("python3 /opt/ros/noetic/bin/roslaunch iiwa_gazebo iiwa_gazebo.launch");

    if(gazeboProcess.waitForStarted()){
      resp.success = true;
      resp.message = "OK";
      gazeboLaunched = true;
    }
    else
    {
      resp.success = false;
      resp.message = "FAILED TO LAUNCH GAZEBO";
    }
  }
  else
  {
    resp.success = false;
    resp.message = "GAZEBO ALREADY ONLINE";
  }

  return true;
}

