#include <QApplication>
#include <QIcon>
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "main_window.h"
#include <thread>


int main(int argc, char *argv[])
{
  //ros::init(argc, argv, "iiwa_gui",ros::init_options::AnonymousName); //if the need for more windows comes
  ros::init(argc,argv,"iiwa_gui");
  usleep(50000);

  QApplication a(argc, argv);

  MainWindow w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString(
                       ros::this_node::getName()));


  w.show();
  return a.exec();
}
