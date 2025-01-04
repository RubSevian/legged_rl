/**
 * @file StaticController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <legged_rl_controller/StaticController.h>
#include <pluginlib/class_list_macros.hpp>


namespace legged{

bool StaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  
  if(!LeggedBaseController::init(robot_hw, controller_nh)){
    return false;
  }

  cosCurves_.resize(jointNum_);

  targetPos_.resize(3);
  targetPos_.at(0) = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                      -0.2, 1.36, -2.65, 0.2, 1.36, -2.65}; // ready
  targetPos_.at(1) = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, 0.0, 0.67, -1.3};  // stand
  targetPos_.at(2) = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                      -0.5, 1.36, -2.65, 0.5, 1.36, -2.65}; // down

  ros::NodeHandle nh;
  targetIdxSub_ = nh.subscribe<std_msgs::Int8>("/target_idx", 1, &StaticController::_targetIdxCallback, this);
  targetIdx_ = 0;
  
  return true;
}

void StaticController::_targetIdxCallback(const std_msgs::Int8::ConstPtr & msg){
  targetIdx_ = msg->data;
  auto t0 = currentTime_;
  auto t1 = currentTime_+trajTime_;

  if (targetIdx_ < 0 || targetIdx_ > 2){
    ROS_ERROR_STREAM("[StaticController] Invalid target index");
    for(size_t i=0; i<jointNum_; i++){
      cosCurves_[i].reset(
        obs_.jointPos[i],
        obs_.jointPos[i],
        t0.toSec(),
        t1.toSec());
    }
  } else {
    ROS_INFO_STREAM("[StaticController] Target index: " << targetIdx_);
    for(size_t i=0; i<jointNum_; i++){
      cosCurves_[i].reset(
        obs_.jointPos[i],
        targetPos_[targetIdx_][i],
        t0.toSec(),
        t1.toSec());
    }
  }
}

void StaticController::starting(const ros::Time& time){
  LeggedBaseController::starting(time);

  currentTime_ = time;
  _updateObservation();
  for(size_t i=0; i<jointNum_; i++){
    cosCurves_[i].reset(
      obs_.jointPos[i],
      targetPos_[targetIdx_][i],
      currentTime_.toSec(),
      (currentTime_+trajTime_).toSec());
  }
}


void StaticController::stopping(const ros::Time& time){
  LeggedBaseController::stopping(time);
}

bool StaticController::_beforeUpdate(const ros::Time& time, const ros::Duration& period){
  if(_runThisLoop(period.toSec(), true)){
    // can run the loop in this update, update the observation
    _updateObservation();
    return true;
  } else {
    // set last command (it is necessary, otherwise the desired cmd would be set to 0 in LeggedHW's read function)
    // In fact, it is similar to a ZOH
    for(size_t i=0; i<jointNum_; i++){
      jointActuatorHandles_[i].setCommand(
        obs_.lastJointPosDes[i],
        obs_.lastJointVelDes[i],
        60.0, 
        5.0, 
        0.0
      );
    }
    return false;
  }
}

void StaticController::_afterUpdate(const ros::Time& time, const ros::Duration& period){
  // record last command
  for(size_t i=0; i<jointNum_; i++){
    obs_.lastJointPosDes[i] = cosCurves_[i].getPos(currentTime_.toSec());
    obs_.lastJointVelDes[i] = cosCurves_[i].getVel(currentTime_.toSec());
  }
}

void StaticController::update(const ros::Time& time, const ros::Duration& period){
  if(!_beforeUpdate(time, period)){
    return;
  }

  currentTime_ = time;
  for(size_t i=0; i<jointNum_; i++){
    jointActuatorHandles_[i].setCommand(
      cosCurves_[i].getPos(currentTime_.toSec()),
      cosCurves_[i].getVel(currentTime_.toSec()),
      60.0, 
      5.0, 
      0.0
    );
  }

  _afterUpdate(time, period);
}


} // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::StaticController, controller_interface::ControllerBase);