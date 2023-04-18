#include "joint_position_ui.h"
#include "ui_joint_position_ui.h"
#include "main_window.h"

JointPositionUI::JointPositionUI(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::JointPositionUI),
  publisher_joint_position_(),
  subscriber_joint_position_(),
  subscriber_joint_torque_(),
  subscriber_joint_velocity_(),
  subscriber_destination_reached_(),
  client_time_to_destination_()
{
  ui->setupUi(this);


  //moveitManager = new MoveitManager();



  publisher_joint_position_.init("iiwa");

  subscriber_joint_position_.init("iiwa",std::bind(&JointPositionUI::callbackJointState,this,std::placeholders::_1));
  subscriber_joint_torque_.init("iiwa",std::bind(&JointPositionUI::callbackJointTorque,this,std::placeholders::_1));
  subscriber_joint_velocity_.init("iiwa",std::bind(&JointPositionUI::callbackJointVelocity,this,std::placeholders::_1));
  subscriber_destination_reached_.init("iiwa",std::bind(&JointPositionUI::callbackDestinationReached,this,std::placeholders::_1));

  //client_time_to_destination_.init("iiwa");

  ss_tool_tip_ << std::fixed << std::setprecision(1);
  ss_joint_torque_ << std::fixed << std::setprecision(1);
  ss_joint_velocity_ << std::fixed << std::setprecision(1);


  //array initialisations for less line consuming access
  sliders_joint_angles[0] = ui->slider_A1;
  sliders_joint_angles[1] = ui->slider_A2;
  sliders_joint_angles[2] = ui->slider_A3;
  sliders_joint_angles[3] = ui->slider_A4;
  sliders_joint_angles[4] = ui->slider_A5;
  sliders_joint_angles[5] = ui->slider_A6;
  sliders_joint_angles[6] = ui->slider_A7;

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

  ui->statesWidget->setCurrentIndex(0);
  connect(parent,SIGNAL(robotStatus(bool)),this,SLOT(robotState(bool)));
  connect(parent,SIGNAL(rvizStatus(bool)),this,SLOT(rvizState(bool)));
}










//destructor
JointPositionUI::~JointPositionUI()
{
  delete ui;
  if(moveitManager != nullptr)
    delete moveitManager;
}











void JointPositionUI::robotState(bool isOnline)
{
  robotOnline = isOnline;
}

void JointPositionUI::rvizState(bool isOnline)
{

  if(isOnline){
    moveitManager = new MoveitManager();
  }
  else{
    if(moveitManager != nullptr)
      moveitManager->~MoveitManager();
  }

  rvizOnline = isOnline;
}























//slider moved slots



void JointPositionUI::on_slider_A1_sliderMoved(int position)
{
  joint_angles[0] = (float)position / 10;


  ss_tool_tip_ << joint_angles[0];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A2_sliderMoved(int position)
{
  joint_angles[1] = (float)position / 10;


  ss_tool_tip_ << joint_angles[1];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A3_sliderMoved(int position)
{
  joint_angles[2] = (float)position / 10;


  ss_tool_tip_ << joint_angles[2];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A4_sliderMoved(int position)
{
  joint_angles[3] = (float)position / 10;


  ss_tool_tip_ << joint_angles[3];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A5_sliderMoved(int position)
{
  joint_angles[4] = (float)position / 10;


  ss_tool_tip_ << joint_angles[4];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A6_sliderMoved(int position)
{
  joint_angles[5] = (float)position / 10;


  ss_tool_tip_ << joint_angles[5];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}


void JointPositionUI::on_slider_A7_sliderMoved(int position)
{
  joint_angles[6] = (float)position / 10;


  ss_tool_tip_ << joint_angles[6];

  QToolTip::showText(QCursor::pos(),QString::fromStdString(ss_tool_tip_.str()),this);
  ss_tool_tip_.str(std::string());
}



























//slider release slots


void JointPositionUI::on_slider_A1_sliderReleased()
{
  ss_tool_tip_ << joint_angles[0];
  ui->slider_A1->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A2_sliderReleased()
{
  ss_tool_tip_ << joint_angles[1];
  ui->slider_A2->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A3_sliderReleased()
{
  ss_tool_tip_ << joint_angles[2];
  ui->slider_A3->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A4_sliderReleased()
{
  ss_tool_tip_ << joint_angles[3];
  ui->slider_A4->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A5_sliderReleased()
{
  ss_tool_tip_ << joint_angles[4];
  ui->slider_A5->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A6_sliderReleased()
{
  ss_tool_tip_ << joint_angles[5];
  ui->slider_A6->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}


void JointPositionUI::on_slider_A7_sliderReleased()
{
  ss_tool_tip_ << joint_angles[6];
  ui->slider_A7->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
  ss_tool_tip_.str(std::string());
  planPosition();
}



















//button slots



void JointPositionUI::on_homePosButton_clicked()
{

  for (size_t i = 0; i < joint_angles.size(); ++i) {
    joint_angles[i] = 0;
    sliders_joint_angles[i]->setValue(0);
    sliders_joint_angles[i]->setToolTip(QString::fromStdString(std::to_string(0)));
  }
  if(rvizOnline)
    moveitManager->planMovementJointPosition(joint_angles);

  setPosition();
}


void JointPositionUI::on_setPosButton_clicked()
{
  setPosition();
}

void JointPositionUI::callbackJointState(const iiwa_msgs::JointPosition &joint_position_){
  emit uiCameToLife(true);

  if((gui_init_ || !destination_reached_) && robotOnline){

    joint_angles[0] = std::round(angles::to_degrees(joint_position_.position.a1));
    joint_angles[1] = std::round(angles::to_degrees(joint_position_.position.a2));
    joint_angles[2] = std::round(angles::to_degrees(joint_position_.position.a3));
    joint_angles[3] = std::round(angles::to_degrees(joint_position_.position.a4));
    joint_angles[4] = std::round(angles::to_degrees(joint_position_.position.a5));
    joint_angles[5] = std::round(angles::to_degrees(joint_position_.position.a6));
    joint_angles[6] = std::round(angles::to_degrees(joint_position_.position.a7));

    for (size_t i = 0; i < sliders_joint_angles.size(); ++i) {
      sliders_joint_angles[i]->setToolTip(QString::fromStdString(std::to_string(joint_angles[i])));
      sliders_joint_angles[i]->setValue(joint_angles[i] * 10);
    }

    gui_init_ = false;
  }
}





















//combobox slots

void JointPositionUI::on_stateSelector_currentIndexChanged(int index)
{
  if(index == 1){
    ui->statesWidget->setCurrentIndex(1);
    return;
  }

  ui->statesWidget->setCurrentIndex(0);
}




















//ROS callbacks



void JointPositionUI::callbackJointTorque(const iiwa_msgs::JointTorque &joint_torque_){

  joint_torques[0] = std::round(joint_torque_.torque.a1 * 10);
  joint_torques[1] = std::round(joint_torque_.torque.a2 * 10);
  joint_torques[2] = std::round(joint_torque_.torque.a3 * 10);
  joint_torques[3] = std::round(joint_torque_.torque.a4 * 10);
  joint_torques[4] = std::round(joint_torque_.torque.a5 * 10);
  joint_torques[5] = std::round(joint_torque_.torque.a6 * 10);
  joint_torques[6] = std::round(joint_torque_.torque.a7 * 10);

  for (size_t i = 0; i < sliders_joint_torques.size(); ++i) {
    sliders_joint_torques[i]->setValue(joint_torques[i]);
    ss_joint_torque_ << ((float)joint_torques[i] / 10);
    labels_joint_torques[i]->setText(QString::fromStdString(ss_joint_torque_.str()));
    ss_joint_torque_.str(std::string());
  }
}

void JointPositionUI::callbackJointVelocity(const iiwa_msgs::JointVelocity &joint_velocity_){

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

void JointPositionUI::callbackDestinationReached(const std_msgs::Time &timestamp){
  destination_reached_ = true;
  ui->setPosButton->setEnabled(true);
}



















//private hepler methods


void JointPositionUI::setPosition(){
  destination_reached_ = false;
  //ui->setPosButton->setEnabled(false);

  iiwa_msgs::JointPosition msg;

  msg.position.a1 = (float) angles::from_degrees((double) joint_angles[0]);
  msg.position.a2 = (float) angles::from_degrees((double) joint_angles[1]);
  msg.position.a3 = (float) angles::from_degrees((double) joint_angles[2]);
  msg.position.a4 = (float) angles::from_degrees((double) joint_angles[3]);
  msg.position.a5 = (float) angles::from_degrees((double) joint_angles[4]);
  msg.position.a6 = (float) angles::from_degrees((double) joint_angles[5]);
  msg.position.a7 = (float) angles::from_degrees((double) joint_angles[6]);

  if(rvizOnline)
    moveitManager->moveToPosition();

  publisher_joint_position_.setPosition(msg);
}

void JointPositionUI::planPosition(){
 if(rvizOnline){
    std::array<float, 7> temp;
    for(int8_t i = 0; i< joint_angles.size(); ++i){
      temp[i] = (float) angles::from_degrees((double) joint_angles[i]);
    }
    moveitManager->planMovementJointPosition(temp);
  }
}







