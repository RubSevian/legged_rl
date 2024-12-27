/**
 * @file LeggedBaseController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <legged_rl_controller/LeggedBaseController.h>

namespace legged{

bool LeggedBaseController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){

  // ************************************************************************************************
  // * Get ROS parameters
  // ************************************************************************************************
  ros::NodeHandle nh;
  ros::NodeHandle nhConfig("robot_config");
  
  if(!controller_nh.getParam("frequency", loopFrequency_)){
    ROS_ERROR_STREAM("[LeggedBaseController] Could not read the frequency parameter");
    return false;
  }

  if(!nhConfig.getParam("joint_names", jointNames_)){
    ROS_ERROR_STREAM("[LeggedBaseController] Could not read the joint_names parameter");
    return false;
  }

  if(!nhConfig.getParam("imu/handle_name", imuHandleName_)){
    ROS_ERROR_STREAM("[LeggedBaseController] Could not read the imu/handle_name parameter");
    return false;
  }

  if(!nhConfig.getParam("hw_loop/loop_frequency", cmFrequency_)){
    ROS_ERROR_STREAM("[LeggedBaseController] Could not read the hw_loop/loop_frequency parameter");
    return false;
  }

  // ************************************************************************************************
  // * Initialize Joints and IMU configuration
  // ************************************************************************************************

  _initJoints(robot_hw);
  _initImu(robot_hw);
  _initObservation();


  // ************************************************************************************************
  // * Loop configuration
  // ************************************************************************************************
  if (loopFrequency_ > cmFrequency_){
    ROS_ERROR_STREAM("[LeggedBaseController] Controller Loop frequency " << loopFrequency_ << " is higher than the control manager frequency " << cmFrequency_);
    return false;
  }
  // double cmPeriod = 1.0 / cmFrequency_;   // e.g. 1/1000=0.001
  // double loopPeriod = 1.0 / loopFrequency_; // e.g. 1/100=0.01
  // loopEveryN_ = static_cast<int>(loopPeriod / cmPeriod);  // e.g. 0.01/0.001=10
  loopEveryN_ = static_cast<int>(cmFrequency_ / loopFrequency_);
  

  return true;

}

void LeggedBaseController::_initJoints(hardware_interface::RobotHW* robot_hw){
  jointNum_ = jointNames_.size();

  // get joint handles from ros control interface
  auto * jointActuatorInterface = robot_hw->get<JointActuatorInterface>();
  for(const auto & jntName : jointNames_){
    JointActuatorHandle jntHandle = jointActuatorInterface->getHandle(jntName);
    jointActuatorHandles_.push_back(jntHandle);
  }
}

void LeggedBaseController::_initImu(hardware_interface::RobotHW* robot_hw){
  // get imu handle from ros control interface
  auto * imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuInterface->getHandle(imuHandleName_);
}

void LeggedBaseController::_initObservation(){
  obs_.jointPos.resize(jointNum_);
  obs_.jointVel.resize(jointNum_);
  obs_.lastJointPosDes.resize(jointNum_);
  obs_.lastJointVelDes.resize(jointNum_);
}

void LeggedBaseController::_updateObservation(){
  // update joint info
  for(size_t i=0; i<jointNum_; i++){
    obs_.jointPos[i] = jointActuatorHandles_[i].getPosition();
    obs_.jointVel[i] = jointActuatorHandles_[i].getVelocity();
  }

  // update imu info
  obs_.imuOri[0] = imuSensorHandle_.getOrientation()[0];  // x
  obs_.imuOri[1] = imuSensorHandle_.getOrientation()[1];  // y
  obs_.imuOri[2] = imuSensorHandle_.getOrientation()[2];  // z
  obs_.imuOri[3] = imuSensorHandle_.getOrientation()[3];  // w

  obs_.imuAngVel[0] = imuSensorHandle_.getAngularVelocity()[0];  // x
  obs_.imuAngVel[1] = imuSensorHandle_.getAngularVelocity()[1];  // y
  obs_.imuAngVel[2] = imuSensorHandle_.getAngularVelocity()[2];  // z

  obs_.imuLinAcc[0] = imuSensorHandle_.getLinearAcceleration()[0];  // x
  obs_.imuLinAcc[1] = imuSensorHandle_.getLinearAcceleration()[1];  // y
  obs_.imuLinAcc[2] = imuSensorHandle_.getLinearAcceleration()[2];  // z
  
}

bool LeggedBaseController::_runThisLoop(double period, bool debug){
  if(loopCounter_ % loopEveryN_ == 0){
    if(debug){
      ROS_INFO_STREAM_THROTTLE(1, "[LeggedBaseController] Running the controller loop at frequency: " << 1.0/timePassed_);
    }
    loopCounter_ = 1;
    timePassed_ = 0.0;
    return true;
  }
  loopCounter_++;
  timePassed_ += period;
  return false;
}

void LeggedBaseController::update(const ros::Time& time, const ros::Duration& period) {
  if(!_runThisLoop(period.toSec())){
    return;
  }
  _updateObservation();
}


void LeggedBaseController::starting(const ros::Time& time) {
  loopCounter_ = loopEveryN_; // so that the controller can run in the first loop
}


void LeggedBaseController::stopping(const ros::Time& time) {

}


}