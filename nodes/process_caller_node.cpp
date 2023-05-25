#include <ros/ros.h>
#include "process_caller.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "process_caller_node");
  ProcessCaller p = ProcessCaller();

  while(ros::ok()){
    ros::spinOnce();
    usleep(10000);
  }

  return 0;
}
