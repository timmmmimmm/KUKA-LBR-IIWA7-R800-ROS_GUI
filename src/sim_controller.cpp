#include "sim_controller.h"

SimController::SimController(QObject *parent)
  : QObject{parent}
{
  rviz_client_ = node_handle_.serviceClient<std_srvs::SetBool>("launch_rviz");
  gazebo_client_ = node_handle_.serviceClient<std_srvs::SetBool>("launch_gazebo");
  sim_stop_client_ = node_handle_.serviceClient<std_srvs::SetBool>("kill_sims");
}

bool SimController::lauchBoth()
{
  std_srvs::SetBool srv;
  srv.request.data = false;

  if(!gazebo_client_.exists()){
    ROS_WARN("Process caller is not online, please run the node!");
    std::stringstream ss;
    ss << "Gazebo server is offline, launch it and try again!";
    emit gazeboFailed(ss.str());
    emit rvizStatus(false);
    return false;
  }

  gazebo_client_.call(srv);


  if(!rviz_client_.exists()){
    ROS_WARN("Process caller is not online, please run the node!");
    std::stringstream ss;
    ss << "Rviz server is offline, launch it and try again!";
    emit rvizFailed(ss.str());
    emit rvizStatus(false);
    return false;
  }


  srv.request.data = true;

  sleep(10);
  emit gazeboStatus(true);

  rviz_client_.call(srv);
  sleep(10);
  emit rvizStatus(true);
  return true;
}

bool SimController::stopSims()
{
  std_srvs::SetBool srv;

  srv.request.data = false;

  sim_stop_client_.call(srv);

  if(!srv.response.success){
                  //process caller node needs an update for this to work (diff GShutown, RShutdown)
    return false;
  }


  emit rvizStatus(false);
  emit gazeboStatus(false);

  return true;
}

bool SimController::launchRviz(bool withGazebo)
{
  if(!rviz_client_.exists()){
    ROS_WARN("Process caller is not online, please run the node!");
    std::stringstream ss;
    ss << "Rviz server is offline, launch it and try again!";
    emit rvizFailed(ss.str());
    return false;
  }

  std_srvs::SetBool srv;

  if(withGazebo)
    srv.request.data = true;
  else
    srv.request.data = false;


  rviz_client_.call(srv);
  sleep(10);
  emit rvizStatus(true);
  return true;
}

bool SimController::launchGazebo()
{
  std_srvs::SetBool srv;
  srv.request.data = false;

  if(!gazebo_client_.exists()){
    ROS_WARN("Process caller is not online, please run the node!");
    std::stringstream ss;
    ss << "Gazebo server is offline, launch it and try again!";
    emit gazeboFailed(ss.str());
    return false;
  }

  gazebo_client_.call(srv);

  sleep(10);
  emit gazeboStatus(true);
  return true;
}


