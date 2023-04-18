#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <thread>
#include <unistd.h>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

bool gazeboLauched = false;
bool rvizLaunched = false;

bool launchGazebo(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  if(!gazeboLauched){
    auto launch = [](){
      std::system("roslaunch iiwa_gazebo iiwa_gazebo.launch");
    };

    std::thread t1 = std::thread(launch);
    t1.detach();

    gazeboLauched = true;
    resp.success = true;
    resp.message = "Success!";
    return true;
  }

  resp.message = "Gazebo already launched!";
  resp.success = false;
  return true;
}

bool launchRviz(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  if(!rvizLaunched){

    if(req.data){

      auto launchRviz = [](){
        std::system("roslaunch iiwa_moveit demo2.launch");
      };

      std::thread t2 = std::thread(launchRviz);
      t2.detach();
    }

    else{

      auto launchRviz = [](){
      std::system("roslaunch iiwa_moveit demo.launch");
      };

      std::thread t2 = std::thread(launchRviz);
      t2.detach();
    }

    rvizLaunched = true;
    resp.success = true;
    resp.message = "Success!";
    return true;
  }

  resp.message = "Rviz already launched!";
  resp.success = false;
  return true;
}

bool killSims(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
  std::string cwd = "iiwa_stack_ws/src/iiwa_gui/scripts/";
  std::ifstream pidFile;
  std::vector<std::string> pids {};
  uint8_t fParam = 9;

  while(true){

    std::system(std::string("./" + cwd + "getPids.sh " + std::to_string(fParam)).c_str());

    pidFile.open(std::string(cwd + "pids.txt"));

    if(!pidFile.is_open()){
      resp.success = false;
      resp.message = "PID file not open";
      return true;
    }

    std::string tempString;
    while (std::getline(pidFile,tempString)) {
      pids.push_back(tempString);
    }

    pidFile.close();

    if(pids[0] == ""){
      pids.clear();
      fParam --;

      if(fParam <= 0){
        resp.success = false;
        resp.message = "FAILED TO KILL";
        return true;
      }

      continue;
    }

    if(strtol(pids[0].c_str(),0,10) < 1000){
      pids.clear();
      fParam --;
      continue;
    }
    break;
  }

  for(size_t i = 0; i < pids.size()-1; i++){
    std::system(std::string("kill -2 " + pids[i]).c_str());
    sleep(1);
  }

  sleep(30);

  rvizLaunched = false;
  gazeboLauched = false;

  resp.success = true;
  resp.message = "Success!";
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_caller_node");
  ros::NodeHandle nh;

  ros::ServiceServer gazeboLaunchServer = nh.advertiseService("launch_gazebo",launchGazebo);
  ros::ServiceServer rvizLauchServer = nh.advertiseService("launch_rviz",launchRviz);
  ros::ServiceServer killSimServer = nh.advertiseService("kill_sims",killSims);

  while(ros::ok()){
    ros::spinOnce();
    usleep(10000);
  }

  return 0;
}
