#include "main_window.h"
#include "ui_main_window.h"



MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  spinner = new QTimer(this);
  connect(spinner,SIGNAL(timeout()),this,SLOT(spinNode()));
  spinner->start(1);



  jointPositionUi = new JointPositionUI(this);

  connect(jointPositionUi,SIGNAL(uiCameToLife(bool)),this,SLOT(uiIsAlive(bool)));

  subscriber_robot_online = node_handle_.subscribe("/iiwa/online",
                                                   1,
                                                   &MainWindow::callbackOnline,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());

  //robot status setup
  //if the icon doesnt show up, change the path to ~/iiwa_stack_ws/src/iiwa_gui/rc/<icon_name>
  QIcon icon = QIcon("/run/user/1000/doc/d666df8e/cCircle.svg");
  ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
  ui->label_online->setText(QString("Offline"));

  //control widget setup
  ui->controlsAndState->addWidget(jointPositionUi);

  //tooltip style setup
  this->setStyleSheet("QToolTip { "
                      "color: rgb(31, 154, 211); "
                      "background-color: rgb(39, 39, 39); "
                      "border: 1px solid white; "
                      "}");

  QTimer::singleShot(2000,this,SLOT(checkIfOnline()));

}

MainWindow::~MainWindow()
{
  delete ui;
  delete spinner;
  delete jointPositionUi;
}

void MainWindow::checkIfOnline()
{
  if(!isRobotOnline){
    popup = new UniversalPopup(this);
    popup->setWindowTitle(QString::fromStdString(std::string("Launch Gazebo?")));
    popup->setMainText("IIWA is not connected yet.\nWould you like to start a simulation?");

    auto icon = std::make_shared<QIcon>("/run/user/1000/doc/d10395e9/gazeboIcon.svg");
    popup->setIcon(icon.get());
    popup->setPositiveButtonText("Launch Simulation");

    popup->setPositiveButtonCallback(std::bind(&MainWindow::launchSim,this,true));
    popup->show();
    popup->exec();
  }

}

void MainWindow::spinNode()
{
  if(!ros::ok)
    QApplication::quit();

  ros::spinOnce();
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
  if(index == 1){
    ui->controlsAndState->setCurrentIndex(1);
    return;
  }
  ui->controlsAndState->setCurrentIndex(0);
}

void MainWindow::uiIsAlive(bool status)
{
  isUIOnline = status;
}

bool MainWindow::launchSim(bool withGazebo)
{
  if(withGazebo){
    auto gazeboClient = node_handle_.serviceClient<std_srvs::SetBool>("launch_gazebo");

    auto rvizClient = node_handle_.serviceClient<std_srvs::SetBool>("launch_rviz");


    auto launchGazebo = [&]() -> bool
    {
      std_srvs::SetBool srv;
      srv.request.data = false;

      if(!gazeboClient.exists()){
        ROS_WARN("Process caller is not online, please run the node!");
        popup->setMainText("");
        std::stringstream ss;
        ss << "Gazebo server is offline, launch it and try again!";
        popup->setMainText(ss.str());
        return false;
      }

      gazeboClient.call(srv);
      return true;
    };

    auto launchRviz = [&]() -> bool
    {
      if(!rvizClient.exists()){
        ROS_WARN("Process caller is not online, please run the node!");
        popup->setMainText("");
        std::stringstream ss;
        ss << "Rviz server is offline, launch it and try again!";
        popup->setMainText(ss.str());
        return false;
      }

      std_srvs::SetBool srv;
      srv.request.data = true;

      sleep(10);
      gazeboOnline = true;
      emit gazeboStatus(true);

      rvizClient.call(srv);
      sleep(5);
      rvizOnline = true;
      emit rvizStatus(true);
      return true;
    };


    std::future<bool> gazRes = std::async(launchGazebo);
    if(!gazRes.get())
      return false;



    std::future<bool> rvizRes = std::async(launchRviz);
    if(!rvizRes.get())
      return false;

    return true;
  }

  auto rvizClient = node_handle_.serviceClient<std_srvs::SetBool>("launch_rviz");

  auto launchRviz = [&]() -> bool
  {

    if(!rvizClient.exists()){
      popup->setMainText("");
      ROS_WARN("Process caller is not online, please run the node!");
      std::stringstream ss;
      ss << "Rviz server is offline, launch it and try again!";
      popup->setMainText(ss.str());
      return false;
    }

    std_srvs::SetBool srv;
    srv.request.data = false;
    rvizClient.call(srv);
    sleep(5);
    return true;
  };

  std::future<bool> t1 = std::async(launchRviz);
  if(!t1.get())
    return false;

  rvizOnline = true;
  emit rvizStatus(true);
  return true;
}


void MainWindow::callbackOnline(const std_msgs::Bool &isOnline){
  if(!this->isRobotOnline && isOnline.data){
    QIcon icon = QIcon("/run/user/1000/doc/c5ae8b17/gCircle.svg");
    ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
    ui->label_online->setText(QString("Online"));
  }

  else if(!isOnline.data && this->isRobotOnline){
    QIcon icon = QIcon("/run/user/1000/doc/d666df8e/cCircle.svg");
    ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
    ui->label_online->setText(QString("Offline"));
  }

  this->isRobotOnline = isOnline.data;
  if(isUIOnline){
    emit robotStatus(isOnline.data);
  }
}


void MainWindow::on_actionLaunch_triggered()
{
  ui->actionLaunch->setEnabled(false);
}

