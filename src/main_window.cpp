#include "main_window.h"
#include "ui_main_window.h"



MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  subscriber_joint_torque_(),
  subscriber_joint_velocity_(),
  subscriber_cartesian_wrench_()
{
  ui->setupUi(this);

  ui->stateSelector->setItemIcon(0, QIcon(":/icons/pCircle.svg"));
  ui->stateSelector->setItemIcon(1, QIcon(":/icons/oCircle.svg"));
  ui->stateSelector->setItemIcon(2, QIcon(":/icons/cCircle.svg"));

  spinner = new QTimer(this);
  connect(spinner,SIGNAL(timeout()),this,SLOT(spinNode()));
  spinner->start(1);


  simController = new SimController();

  connect(simController,SIGNAL(gazeboStatus(bool)), this, SLOT(gazeboOperational(bool)));
  connect(simController,SIGNAL(rvizStatus(bool)), this, SLOT(rvizOperational(bool)));

  ss_joint_torque_ << std::fixed << std::setprecision(1);
  ss_joint_velocity_ << std::fixed << std::setprecision(1);
  ss_cartesian_force << std::fixed << std::setprecision(2);
  ss_cartesian_torque << std::fixed << std::setprecision(2);

  subscriber_joint_torque_.init("iiwa",std::bind(&MainWindow::callbackJointTorque,this,std::placeholders::_1));
  subscriber_joint_velocity_.init("iiwa",std::bind(&MainWindow::callbackJointVelocity,this,std::placeholders::_1));
  subscriber_cartesian_wrench_.init("iiwa",std::bind(&MainWindow::callbackCartesianWrech,this,std::placeholders::_1));

  jointPositionUi = new JointPositionUI(this);
  jointPositionUi->setFocusedWidget(true);
  cartesianPositionUi = new CartesianPositionUi(this);

  connect(jointPositionUi,SIGNAL(jointPositionUIExists(bool)),this,SLOT(jointPositionUICreated(bool)));
  connect(cartesianPositionUi,SIGNAL(cartesianPositionUIExists(bool)),this,SLOT(cartesianPositionUiCreated(bool)));
  connect(cartesianPositionUi,SIGNAL(setHome()), this, SLOT(setHomeRequested()));


  subscriber_robot_online = node_handle_.subscribe("/iiwa/online",
                                                   1,
                                                   &MainWindow::callbackOnline,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay());

  speedControlClient = node_handle_.serviceClient<iiwa_msgs::SetSmartServoJointSpeedLimits>
      ("/iiwa/configuration/setSmartServoLimits");

  //robot status setup
  //if the icon doesnt show up, change the path to ~/iiwa_stack_ws/src/iiwa_gui/rc/<icon_name>
  QIcon icon = QIcon(":/icons/cCircle.svg");
  ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
  ui->label_online->setText(QString("Offline"));

  //control widget setup
  ui->controlsAndState->addWidget(jointPositionUi);
  ui->controlsAndState->addWidget(cartesianPositionUi);

  //tooltip style setup
  this->setStyleSheet("QToolTip { "
                      "color: rgb(31, 154, 211); "
                      "background-color: rgb(39, 39, 39); "
                      "border: 1px solid white; "
                      "}");




  //Uncomment when moveit works with Gazebo
  //QTimer::singleShot(2000,this,SLOT(checkIfOnline()));








  sliders_joint_torques[0] = ui->sliderA1Torque;
  sliders_joint_torques[1] = ui->sliderA2Torque;
  sliders_joint_torques[2] = ui->sliderA3Torque;
  sliders_joint_torques[3] = ui->sliderA4Torque;
  sliders_joint_torques[4] = ui->sliderA5Torque;
  sliders_joint_torques[5] = ui->sliderA6Torque;
  sliders_joint_torques[6] = ui->sliderA7Torque;

  labels_joint_torques[0] = ui->labelA1Torque;
  labels_joint_torques[1] = ui->labelA2Torque;
  labels_joint_torques[2] = ui->labelA3Torque;
  labels_joint_torques[3] = ui->labelA4Torque;
  labels_joint_torques[4] = ui->labelA5Torque;
  labels_joint_torques[5] = ui->labelA6Torque;
  labels_joint_torques[6] = ui->labelA7Torque;

  sliders_joint_velocities[0] = ui->sliderA1Velocity;
  sliders_joint_velocities[1] = ui->sliderA2Velocity;
  sliders_joint_velocities[2] = ui->sliderA3Velocity;
  sliders_joint_velocities[3] = ui->sliderA4Velocity;
  sliders_joint_velocities[4] = ui->sliderA5Velocity;
  sliders_joint_velocities[5] = ui->sliderA6Velocity;
  sliders_joint_velocities[6] = ui->sliderA7Velocity;

  labels_joint_velocities[0] = ui->labelA1Velocity;
  labels_joint_velocities[1] = ui->labelA2Velocity;
  labels_joint_velocities[2] = ui->labelA3Velocity;
  labels_joint_velocities[3] = ui->labelA4Velocity;
  labels_joint_velocities[4] = ui->labelA5Velocity;
  labels_joint_velocities[5] = ui->labelA6Velocity;
  labels_joint_velocities[6] = ui->labelA7Velocity;

  sliders_cartesian_force[0] = ui->sliderXForce;
  sliders_cartesian_force[1] = ui->sliderYForce;
  sliders_cartesian_force[2] = ui->sliderZForce;
  sliders_cartesian_torque[0] = ui->sliderXTorque;
  sliders_cartesian_torque[1] = ui->sliderYTorque;
  sliders_cartesian_torque[2] = ui->sliderZTorque;

  labels_cartesian_force[0].first = ui->labelXForce;
  labels_cartesian_force[0].second = ui->labelXForceError;
  labels_cartesian_force[1].first = ui->labelYForce;
  labels_cartesian_force[1].second = ui->labelYForceError;
  labels_cartesian_force[2].first = ui->labelZForce;
  labels_cartesian_force[2].second = ui->labelZForceError;

  labels_cartesian_torque[0].first = ui->labelXTorque;
  labels_cartesian_torque[0].second = ui->labelXTorqueError;
  labels_cartesian_torque[1].first = ui->labelYTorque;
  labels_cartesian_torque[1].second = ui->labelYTorqueError;
  labels_cartesian_torque[2].first = ui->labelZTorque;
  labels_cartesian_torque[2].second = ui->labelZTorqueError;



  ui->statesWidget->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
  delete jointPositionUi;
  delete cartesianPositionUi;
  delete simController;
  delete ui;
  delete spinner;

  if(moveitManager != nullptr){
    delete moveitManager;
  }
}






void MainWindow::checkIfOnline()
{
  if(!isRobotOnline){
    auto icon = std::make_shared<QIcon>(":/icons/gazeboIcon.svg");

    createUniversalPopup("Launch Gazebo?",
                         "IIWA is not connected yet.\nWould you like to start a simulation?",
                         "Launch Simulation",
                         icon.get(),
                         std::bind(&SimController::lauchBoth,simController));
  }

}






//QTimer spinner

void MainWindow::spinNode()
{
  if(!ros::ok)
    QApplication::quit();

  ros::spinOnce();
}










//Slots
void MainWindow::rvizOperational(bool status)
{
  if(status)
  {
    ui->actionRvizNo_Sim->setEnabled(false);
    ui->actionRvizNo_Sim->setVisible(false);
    ui->actionRvizWith_Sim->setEnabled(false);
    ui->actionRvizWith_Sim->setVisible(false);

    ui->menuRvizLaunch->setEnabled(false);
    ui->menuRvizLaunch->setVisible(false);

    ui->actionCloseRviz->setEnabled(true);
    ui->actionCloseRviz->setVisible(true);

    ui->menuRviz->setIcon(QIcon(":/icons/gCircle.svg"));

    moveitManager = new MoveitManager(gazeboOnline);
    jointPositionUi->setMoveitManager(moveitManager);
    cartesianPositionUi->setMoveitManager(moveitManager);
  }
  else
  {
    ui->actionRvizNo_Sim->setEnabled(true);
    ui->actionRvizNo_Sim->setVisible(true);
    ui->actionRvizWith_Sim->setEnabled(true);
    ui->actionRvizWith_Sim->setVisible(true);

    ui->menuRvizLaunch->setEnabled(true);
    ui->menuRvizLaunch->setVisible(true);

    ui->actionCloseRviz->setEnabled(false);
    ui->actionCloseRviz->setVisible(false);

    ui->menuRviz->setIcon(QIcon());

    if(moveitManager != nullptr){
      delete moveitManager;
      moveitManager = nullptr;
    }
  }

  rvizOnline = status;
  emit rvizStatus(status);
}

void MainWindow::gazeboOperational(bool status)
{
  gazeboOnline = status;
  emit gazeboStatus(status);

  if(gazeboOnline){
    ui->actionLaunchGazebo->setEnabled(false);
    ui->actionLaunchGazebo->setVisible(false);

    ui->actionCloseGazebo->setEnabled(true);
    ui->actionCloseGazebo->setVisible(true);

    ui->menuGazebo->setIcon(QIcon(":/icons/gCircle.svg"));
    return;
  }

  ui->actionLaunchGazebo->setEnabled(true);
  ui->actionLaunchGazebo->setVisible(true);

  ui->actionCloseGazebo->setEnabled(false);
  ui->actionCloseGazebo->setVisible(false);

  ui->menuGazebo->setIcon(QIcon());

}

void MainWindow::setHomeRequested()
{
  emit setHome();
}


void MainWindow::jointPositionUICreated(bool status)
{
  jointPositionUICreated_ = status;
}

void MainWindow::cartesianPositionUiCreated(bool status)
{
  cartesianPositionUiCreated_ = status;
}






















//ROS robot online callback
void MainWindow::callbackOnline(const std_msgs::Bool &isOnline){
  if(!this->isRobotOnline && isOnline.data){
    QIcon icon = QIcon(":/icons/gCircle.svg");
    ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
    ui->label_online->setText(QString("Online"));

    initializeRobotSpeed();
  }

  else if(!isOnline.data && this->isRobotOnline){
    QIcon icon = QIcon(":/icons/cCircle.svg");
    ui->label_icon->setPixmap(icon.pixmap(QSize(16,16)));
    ui->label_online->setText(QString("Offline"));
  }

  this->isRobotOnline = isOnline.data;

  if(jointPositionUICreated_ && cartesianPositionUiCreated_){
    emit robotStatus(isOnline.data);
  }
}


void MainWindow::callbackJointTorque(const iiwa_msgs::JointTorque &joint_torque_){

  joint_torques[0] = std::round(joint_torque_.torque.a1 * 10);
  joint_torques[1] = std::round(joint_torque_.torque.a2 * 10);
  joint_torques[2] = std::round(joint_torque_.torque.a3 * 10);
  joint_torques[3] = std::round(joint_torque_.torque.a4 * 10);
  joint_torques[4] = std::round(joint_torque_.torque.a5 * 10);
  joint_torques[5] = std::round(joint_torque_.torque.a6 * 10);
  joint_torques[6] = std::round(joint_torque_.torque.a7 * 10);

  for (size_t i = 0; i < sliders_joint_torques.size(); ++i) {
    sliders_joint_torques[i]->setValue(joint_torques[i]);
    ss_joint_torque_ << ((float)joint_torques[i]/10);
    labels_joint_torques[i]->setText(QString::fromStdString(ss_joint_torque_.str()));
    ss_joint_torque_.str(std::string());
  }
}

void MainWindow::callbackJointVelocity(const iiwa_msgs::JointVelocity &joint_velocity_){

  joint_velocities[0] = std::round(joint_velocity_.velocity.a1 * 10000);
  joint_velocities[1] = std::round(joint_velocity_.velocity.a2 * 10000);
  joint_velocities[2] = std::round(joint_velocity_.velocity.a3 * 10000);
  joint_velocities[3] = std::round(joint_velocity_.velocity.a4 * 10000);
  joint_velocities[4] = std::round(joint_velocity_.velocity.a5 * 10000);
  joint_velocities[5] = std::round(joint_velocity_.velocity.a6 * 10000);
  joint_velocities[6] = std::round(joint_velocity_.velocity.a7 * 10000);

  for(size_t i = 0; i < sliders_joint_velocities.size(); ++i){
    sliders_joint_velocities[i]->setValue(joint_velocities[i]);
    ss_joint_velocity_ << ((float)joint_velocities[i] / 10);
    labels_joint_velocities[i]->setText(QString::fromStdString(ss_joint_velocity_.str()));
    ss_joint_velocity_.str(std::string());
  }
}

void MainWindow::callbackCartesianWrech(const iiwa_msgs::CartesianWrench &cartesian_wrench_)
{
  cartesian_force[0].first = std::round(cartesian_wrench_.wrench.force.x * 10);
  cartesian_force[1].first = std::round(cartesian_wrench_.wrench.force.y * 10);
  cartesian_force[2].first = std::round(cartesian_wrench_.wrench.force.z * 10);
  cartesian_force[0].second = cartesian_wrench_.inaccuracy.force.x;
  cartesian_force[1].second = cartesian_wrench_.inaccuracy.force.y;
  cartesian_force[2].second = cartesian_wrench_.inaccuracy.force.z;

  cartesian_torque[0].first = std::round(cartesian_wrench_.wrench.torque.x * 10);
  cartesian_torque[1].first = std::round(cartesian_wrench_.wrench.torque.y * 10);
  cartesian_torque[2].first = std::round(cartesian_wrench_.wrench.torque.z * 10);
  cartesian_torque[0].second = cartesian_wrench_.inaccuracy.torque.x;
  cartesian_torque[1].second = cartesian_wrench_.inaccuracy.torque.y;
  cartesian_torque[2].second = cartesian_wrench_.inaccuracy.torque.z;

  for(int8_t i = 0; i < 3; ++i){
    sliders_cartesian_force[i]->setValue(cartesian_force[i].first);
    sliders_cartesian_torque[i]->setValue(cartesian_torque[i].first);

    ss_cartesian_force << ((float) cartesian_force[i].first / 10);
    labels_cartesian_force[i].first->setText(QString::fromStdString(ss_cartesian_force.str()));
    ss_cartesian_force.str(std::string());

    ss_cartesian_force << "+-" << cartesian_force[i].second;
    labels_cartesian_force[i].second->setText(QString::fromStdString(ss_cartesian_force.str()));
    ss_cartesian_force.str(std::string());

    ss_cartesian_torque << ((float) cartesian_torque[i].first / 10);
    labels_cartesian_torque[i].first->setText(QString::fromStdString(ss_cartesian_torque.str()));
    ss_cartesian_torque.str(std::string());

    ss_cartesian_torque << "+-" << cartesian_torque[i].second;
    labels_cartesian_torque[i].second->setText(QString::fromStdString(ss_cartesian_torque.str()));
    ss_cartesian_torque.str(std::string());
  }
}




















//Combobox callback
void MainWindow::on_comboBox_currentIndexChanged(int index)
{
  cartesianPositionUi->setFocusedWidget(false);
  jointPositionUi->setFocusedWidget(false);

  if(index == 1){
    if(!rvizOnline){
      auto icon = std::make_shared<QIcon>(":/icons/rvizIcon.png");
      createUniversalPopup("Open Rviz",
                           "Cartesian coordinates might be confusing, please use Rviz so you can better understand what you are doing.",
                           "Launch Rviz",
                           icon.get(),
                           std::bind(&SimController::launchRviz,simController,false));

    }
    cartesianPositionUi->setFocusedWidget(true);
    ui->controlsAndState->setCurrentIndex(1);
    return;
  }
  jointPositionUi->setFocusedWidget(true);
  ui->controlsAndState->setCurrentIndex(0);
}

void MainWindow::on_stateSelector_currentIndexChanged(int index)
{
  if(index == 1){
    ui->statesWidget->setCurrentIndex(1);
    return;
  }
  else if(index == 2){
    ui->statesWidget->setCurrentIndex(2);
    return;
  }

  ui->statesWidget->setCurrentIndex(0);
}
























//Sim menu buttons
void MainWindow::on_actionRvizWith_Sim_triggered()
{
//  if(!gazeboOnline){

//    auto icon = std::make_shared<QIcon>("/run/user/1000/doc/d10395e9/gazeboIcon.svg");

//    createUniversalPopup("Launch Gazebo?",
//                         "Gazebo must be launched first!",
//                         "Launch Simulation",
//                         icon.get(),
//                         std::bind(&SimController::lauchBoth,simController));
//  }
//  else{
//    createLoadingPopup("Launching Rviz",
//                       "Please wait while Rviz is being launched",
//                       std::bind(&SimController::launchRviz, simController, true));
//  }
}


void MainWindow::on_actionRvizNo_Sim_triggered()
{
  createLoadingPopup("Launching Rviz",
                     "Please wait while Rviz is being launched",
                     std::bind(&SimController::launchRviz, simController, false));
}
void MainWindow::on_actionCloseRviz_triggered()
{
  createLoadingPopup("Closing all simulation and visualisation software",
                     "Please be patient.",
                     std::bind(&SimController::stopSims, simController));
}


void MainWindow::on_actionLaunchGazebo_triggered()
{
//  createLoadingPopup("Launching Gazebo",
//                     "Please wait while Gazebo is being launched",
//                     std::bind(&SimController::launchGazebo, simController));
}


void MainWindow::on_actionCloseGazebo_triggered()
{
//  createLoadingPopup("Closing all simulation and visualisation software",
//                     "Please be patient.",
//                     std::bind(&SimController::stopSims, simController));
}















//Cleanup when closed
void MainWindow::closeEvent(QCloseEvent *event)
{
  shutdownProcedure = true;
  if(rvizOnline || gazeboOnline){
    createLoadingPopup("Shutting down",
                       "Closing simulation and visualisation software.",
                       std::bind(&SimController::stopSims,simController));
  }

  event->accept();
}




























//Popup helpers

void MainWindow::createUniversalPopup(std::string &&windowName,
                                      std::string &&prompt,
                                      std::string &&positiveButtonText,
                                      QIcon *icon,
                                      std::function<bool ()> callable)
{
  popup = new UniversalPopup(this, simController);
  popup->setWindowTitle(QString::fromStdString(windowName));
  popup->setMainText(prompt);

  popup->setIcon(icon);
  popup->setPositiveButtonText(positiveButtonText);

  popup->setPositiveButtonCallback(callable);
  popup->show();
  popup->exec();
}

void MainWindow::createLoadingPopup(std::string &&windowName,
                                    std::string &&prompt,
                                    std::function<bool()> callable)
{
  popup = new UniversalPopup(this,simController);

  popup->setWindowTitle(QString::fromStdString(windowName));
  popup->setMainText(prompt);
  popup->setIcon(nullptr);
  popup->setPositiveButtonVisibility(false);
  popup->setNegativeButtonVisibility(false);
  popup->executeTask(callable);
  popup->show();
  popup->exec();
}

void MainWindow::initializeRobotSpeed()
{
  if(speedControlClient.exists()){
    iiwa_msgs::SetSmartServoJointSpeedLimits msg;

    msg.request.joint_relative_acceleration = 0.5;
    msg.request.joint_relative_velocity = 0.2;
    msg.request.override_joint_acceleration = 0.5;

    speedControlClient.call(msg);
  }
  else{
    usleep(100000);
    initializeRobotSpeed();
  }
}


