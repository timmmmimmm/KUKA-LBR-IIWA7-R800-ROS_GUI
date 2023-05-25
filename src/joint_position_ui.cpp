#include "joint_position_ui.h"
#include "ui_joint_position_ui.h"

JointPositionUI::JointPositionUI(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::JointPositionUI),
  publisher_joint_position_(),
  subscriber_joint_position_(),
  subscriber_destination_reached_(),
  client_time_to_destination_()
{
  ui->setupUi(this);


  publisher_joint_position_.init("iiwa");

  subscriber_joint_position_.init("iiwa",std::bind(&JointPositionUI::callbackJointState,this,std::placeholders::_1));

  subscriber_destination_reached_.init("iiwa",std::bind(&JointPositionUI::callbackDestinationReached,this,std::placeholders::_1));

  //client_time_to_destination_.init("iiwa");

  subscriber_rviz_joint_state_ = node_handle_.subscribe("/iiwa/joint_states",
                                                        1,
                                                        &JointPositionUI::syncPosition,
                                                        this,
                                                        ros::TransportHints().tcpNoDelay());

  ss_tool_tip_ << std::fixed << std::setprecision(1);


  //array initialisations for less line consuming access
  sliders_joint_angles[0] = ui->slider_A1;
  sliders_joint_angles[1] = ui->slider_A2;
  sliders_joint_angles[2] = ui->slider_A3;
  sliders_joint_angles[3] = ui->slider_A4;
  sliders_joint_angles[4] = ui->slider_A5;
  sliders_joint_angles[5] = ui->slider_A6;
  sliders_joint_angles[6] = ui->slider_A7;



  connect(parent,SIGNAL(robotStatus(bool)),this,SLOT(robotState(bool)));
  connect(parent,SIGNAL(rvizStatus(bool)),this,SLOT(rvizState(bool)));
}










//destructor
JointPositionUI::~JointPositionUI()
{
  delete ui;
}




void JointPositionUI::setMoveitManager(MoveitManager *moveitManager)
{
  this->moveitManager = moveitManager;

  sync = true;
}

void JointPositionUI::setFocusedWidget(bool focus)
{
  hasFocus = focus;
  sync = true;

  if(focus)
    gui_init_ = true;
}











void JointPositionUI::robotState(bool isOnline)
{
  robotOnline = isOnline;
}

void JointPositionUI::rvizState(bool isOnline)
{

  if(isOnline){
    moveitManager->setStartPosition(joint_angles);
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
  emit jointPositionUIExists(true);

  if((gui_init_ || !destination_reached_) && robotOnline && hasFocus){

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
























void JointPositionUI::callbackDestinationReached(const std_msgs::Time &timestamp){
  destination_reached_ = true;
  ui->setPosButton->setEnabled(true);
}



















//private hepler methods


void JointPositionUI::setPosition(){
  destination_reached_ = false;

  if(robotOnline)
    ui->setPosButton->setEnabled(false);

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
    moveitManager->planMovementJointPosition(joint_angles);
 }
}

void JointPositionUI::syncPosition(const sensor_msgs::JointState &joint_state_)
{
  if(rvizOnline && !robotOnline && sync){
    for (int_fast8_t i = 0; i < 7; i++) {
      joint_angles[i] = (float) angles::to_degrees(joint_state_.position.at(i));
      sliders_joint_angles[i]->setValue(std::round(joint_angles[i] * 10));

      ss_tool_tip_ << joint_angles[i];
      sliders_joint_angles[i]->setToolTip(QString::fromStdString(ss_tool_tip_.str()));
      ss_tool_tip_.str(std::string());
    }

    sync = false;
  }
}







