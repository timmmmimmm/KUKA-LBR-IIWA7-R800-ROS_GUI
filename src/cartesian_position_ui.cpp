#include "cartesian_position_ui.h"
#include "ui_cartesian_position_ui.h"

CartesianPositionUi::CartesianPositionUi(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::CartesianPositionUi),
  publisher_cartesian_pose_(),
  publisher_cartesian_pose_linear_(),
  subscriber_cartesian_pose_(),
  subscriber_destination_reached_()
{
  ui->setupUi(this);
  publisher_cartesian_pose_.init("iiwa");
  publisher_cartesian_pose_linear_.init("iiwa");
  subscriber_cartesian_pose_.init("iiwa", std::bind(&CartesianPositionUi::callbackCartesianPose, this, std::placeholders::_1));
  subscriber_destination_reached_.init("iiwa", std::bind(&CartesianPositionUi::callbackDestinationReached, this, std::placeholders::_1));

  subscriber_cartesian_pose_angles_ = node_handle_.subscribe("/iiwa/abc_angles",
                                                             1,
                                                             &CartesianPositionUi::callbackCartesianPoseAngles,
                                                             this,
                                                             ros::TransportHints().tcpNoDelay());

  positionSpinBoxes[0] = ui->spinBoxPositionX;
  positionSpinBoxes[1] = ui->spinBoxPositionY;
  positionSpinBoxes[2] = ui->spinBoxPositionZ;

  orientationSpinBoxes[0] = ui->spinBoxOrientationA;
  orientationSpinBoxes[1] = ui->spinBoxOrientationB;
  orientationSpinBoxes[2] = ui->spinBoxOrientationC;

  connect(parent,SIGNAL(robotStatus(bool)),this,SLOT(robotState(bool)));
  connect(parent,SIGNAL(rvizStatus(bool)),this,SLOT(rvizState(bool)));

  emit cartesianPositionUIExists(true);
}

CartesianPositionUi::~CartesianPositionUi()
{
  delete ui;
}

void CartesianPositionUi::setMoveitManager(MoveitManager *moveitManager)
{
  this->moveitManager = moveitManager;

  if(!robotOnline && rvizOnline){
    auto rvizPose = moveitManager->getCurrentPose();
    for(int_fast8_t i = 0; i < 3; ++i){
      positions[i] = rvizPose->at(i).first;
      positionSpinBoxes[i]->setValue(positions[i] * 1000);

      orientations[i] = rvizPose->at(i).second;
      orientationSpinBoxes[i]->setValue(orientations[i]);
    }
  }
}

void CartesianPositionUi::setFocusedWidget(bool focus)
{
  hasFocus = focus;

  if(rvizOnline && !robotOnline){

    auto rvizPose = moveitManager->getCurrentPose();

    for(int_fast8_t i = 0; i < 3; ++i){
      positions[i] = rvizPose->at(i).first;
      positionSpinBoxes[i]->setValue(positions[i] * 1000);

      orientations[i] = rvizPose->at(i).second;
      orientationSpinBoxes[i]->setValue(orientations[i]);
    }
  }

  if(focus){
    initPos = true;
    initRot = true;
  }
}

void CartesianPositionUi::robotState(bool isOnline)
{
  robotOnline = isOnline;
}

void CartesianPositionUi::rvizState(bool isOnline)
{
  rvizOnline = isOnline;
}
























//Button Slots

void CartesianPositionUi::on_homePosButton_clicked()
{
  std::array<float, 7> defaultPos = {0,0,0,0,0,0,0};

  if(rvizOnline){

    moveitManager->planMovementJointPosition(defaultPos);

    moveitManager->moveToPosition();

    positions[0] = 0;
    positions[1] = 0;
    positions[2] = 1.266; //default ZPos in [m]

    for(int_fast8_t i = 0; i < 3; ++i){
      orientations[i] = 0;
      orientationSpinBoxes[i]->setValue(orientations[i]);

      positionSpinBoxes[i]->setValue(positions[i] * 1000.0);
    }
  }

}


void CartesianPositionUi::on_setPosButton_clicked()
{
  if(rvizOnline)
    moveitManager->moveToPosition();


  if(robotOnline){
   // ui->setPosButton->setEnabled(false);

    geometry_msgs::PoseStamped msg;

    msg.header.frame_id = "iiwa_link_0";

    msg.pose.position.x = positions[0];
    msg.pose.position.y = positions[1];
    msg.pose.position.z = positions[2];

    tf::Quaternion quat;

    quat.setRPY(angles::from_degrees(orientations[2]), angles::from_degrees(orientations[1]), angles::from_degrees(orientations[0]));

    msg.pose.orientation.x = quat.x() * -1;
    msg.pose.orientation.y = quat.y() * -1;
    msg.pose.orientation.z = quat.z() * -1;
    msg.pose.orientation.w = quat.w() * -1;

    ROS_ERROR_STREAM("\nPOS:\n" <<
                     "X: " << msg.pose.position.x << "\n" <<
                     "Y: " << msg.pose.position.y << "\n" <<
                     "Z: " << msg.pose.position.z << "\n" <<
                     "\nROT:\n" <<
                     "X: " << msg.pose.orientation.x << "\n" <<
                     "Y: " << msg.pose.orientation.y << "\n" <<
                     "Z: " << msg.pose.orientation.z << "\n" <<
                     "W: " << msg.pose.orientation.w << "\n");

    publisher_cartesian_pose_.setPose(msg);
  }
}





















//Edit finished slots
void CartesianPositionUi::on_spinBoxPositionX_editingFinished()
{
  planPosition();
}


void CartesianPositionUi::on_spinBoxPositionY_editingFinished()
{
  planPosition();
}


void CartesianPositionUi::on_spinBoxPositionZ_editingFinished()
{
  planPosition();
}


void CartesianPositionUi::on_spinBoxOrientationA_editingFinished()
{
  planPosition();
}


void CartesianPositionUi::on_spinBoxOrientationB_editingFinished()
{
  planPosition();
}


void CartesianPositionUi::on_spinBoxOrientationC_editingFinished()
{
  planPosition();
}



















//Value changed slots
void CartesianPositionUi::on_spinBoxPositionX_valueChanged(double arg1)
{
  positions[0] = arg1/1000;
}


void CartesianPositionUi::on_spinBoxPositionY_valueChanged(double arg1)
{
  positions[1] = arg1/1000;
}


void CartesianPositionUi::on_spinBoxPositionZ_valueChanged(double arg1)
{
  positions[2] = arg1/1000;
}


void CartesianPositionUi::on_spinBoxOrientationA_valueChanged(double arg1)
{
  orientations[0] = arg1;
}


void CartesianPositionUi::on_spinBoxOrientationB_valueChanged(double arg1)
{
  orientations[1] = arg1;
}


void CartesianPositionUi::on_spinBoxOrientationC_valueChanged(double arg1)
{
  orientations[2] = arg1;
}

void CartesianPositionUi::planPosition()
{
  if(rvizOnline)
    moveitManager->planCarteianPose(positions,orientations);
}

void CartesianPositionUi::updateGUIDisplayPose(std::array<double, 3> position, std::array<double, 3> orientation)
{

}










void CartesianPositionUi::callbackCartesianPose(const iiwa_msgs::CartesianPose &pose)
{
  emit cartesianPositionUIExists(true);

  if(initPos && robotOnline && hasFocus){
    positions[0] = pose.poseStamped.pose.position.x;
    positions[1] = pose.poseStamped.pose.position.y;
    positions[2] = pose.poseStamped.pose.position.z;

    for(int_fast8_t i = 0; i < 3; ++i){
      positionSpinBoxes[i]->setValue(positions[i] * 1000);
    }

    initPos = false;
  }
}



void CartesianPositionUi::callbackDestinationReached(const std_msgs::Time &timeStamp)
{
  destinationReached = true;
  ui->setPosButton->setEnabled(true);
}

void CartesianPositionUi::callbackCartesianPoseAngles(const std_msgs::String &abcString)
{
  if(initRot && robotOnline && hasFocus){
    std::stringstream ss;

    double a, b, c;

    ss << abcString.data;

    ss >> a >> b >> c;

    orientations[0] = angles::to_degrees(a);
    orientations[1] = angles::to_degrees(b);
    orientations[2] = angles::to_degrees(c);

    for(int_fast8_t i = 0; i < 3; ++i){
      orientationSpinBoxes[i]->setValue(orientations[i]);
    }

    initRot = false;
  }
}

