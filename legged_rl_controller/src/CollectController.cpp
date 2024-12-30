/***********************************************************************************
 * @file CollectController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-30
 * 
 * @copyright Copyright (c) 2024
 * 
 ***********************************************************************************/

#include <legged_rl_controller/CollectController.h>
#include <pluginlib/class_list_macros.hpp>

#include <sensor_msgs/JointState.h>

namespace legged{

bool CollectController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  
  if(!LeggedBaseController::init(robot_hw, controller_nh)){
    return false;
  }

  cosCurves_.resize(jointNum_);

  trajTime_ = ros::Duration(3.0);

  targetNum_ = 4;
  targetPos_.resize(targetNum_);
  targetPos_.at(0) = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                      -0.2, 1.36, -2.65, 0.2, 1.36, -2.65}; // ready
  targetPos_.at(1) = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, 0.0, 0.67, -1.3};  // stand
  targetPos_.at(2) = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                      -0.5, 1.36, -2.65, 0.5, 1.36, -2.65}; // down
  targetPos_.at(3) = {-0.2, 0.5, -1.0, 0.2, 0.5, -1.0,
                      -0.4, 0.8, -1.0, 0.4, 0.8, -1.0};  // stand

  // random noise generator
  gen_ = std::mt19937(rd_());
  noiseVariance_ = 0.5;
  normalDist_ = std::normal_distribution<double>(0.0, noiseVariance_);
  noiseScale_ = 0.5;

  // Joint PD gains
  jointKp_ = 20.0;
  jointKd_ = 0.5;

  jointUpLimits_.reserve(jointNum_);
  jointLowLimits_.reserve(jointNum_);

  ros::NodeHandle nh;
  _loadUrdf(nh);

  actuatorStatePub_ = nh.advertise<legged_rl_controller::ActuatorState>("actuator_state", 1);

  return true;
}


void CollectController::starting(const ros::Time& time){
  LeggedBaseController::starting(time);

  _updateObservation();

  targetIdx_ = 0;

  for(size_t i=0; i<jointNum_; i++){
    double noise = noiseScale_* normalDist_(gen_);

    cosCurves_[i].reset(
      obs_.jointPos[i],
      targetPos_[targetIdx_][i] + noise,
      time.toSec(),
      (time+trajTime_).toSec());
  }
}


void CollectController::stopping(const ros::Time& time){
  LeggedBaseController::stopping(time);
}

bool CollectController::_beforeUpdate(const ros::Time& time, const ros::Duration& period){
  if(_runThisLoop(period.toSec(), false)){
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
        jointKp_, 
        jointKd_, 
        0.0
      );
    }
    return false;
  }
}

void CollectController::_afterUpdate(const ros::Time& time, const ros::Duration& period){

  double targetJointPos;

  legged_rl_controller::ActuatorState asMsg;
  asMsg.header.stamp = time;

  for(size_t i=0; i<jointNum_; i++){
    targetJointPos = std::clamp(cosCurves_[i].getPos(time.toSec()), jointLowLimits_[i], jointUpLimits_[i]);
    jointActuatorHandles_[i].setCommand(
      targetJointPos,
      0.0,
      jointKp_, 
      jointKd_, 
      0.0
    );

    // record last command
    obs_.lastJointPosDes[i] = targetJointPos;
    obs_.lastJointVelDes[i] = 0.0;

    // publish msg for training actuator network
    asMsg.name.push_back(jointNames_[i]);
    asMsg.pos_des.push_back(targetJointPos);
    asMsg.vel_des.push_back(0.0);
    asMsg.pos.push_back(obs_.jointPos[i]);
    asMsg.vel.push_back(obs_.jointVel[i]);
  }

  asMsg.kp = jointKp_;
  asMsg.kd = jointKd_;
  actuatorStatePub_.publish(asMsg);

}

void CollectController::update(const ros::Time& time, const ros::Duration& period){
  if(!_beforeUpdate(time, period)){
    return;
  }

  // check if time > cosCurves_[i].getEndTime()
  // if yes, update the targetIdx_ and reset the cosCurves
  if(time > ros::Time(cosCurves_[0].getEndTime())){
    targetIdx_ = (targetIdx_+1) % targetNum_;
    for(size_t i=0; i<jointNum_; i++){
      double noise = noiseScale_* normalDist_(gen_);

      cosCurves_[i].reset(
        obs_.jointPos[i],
        targetPos_[targetIdx_][i] + noise,
        time.toSec(),
        (time+trajTime_).toSec());
    }
    ROS_INFO("Target index: %d", targetIdx_);
  }

  _afterUpdate(time, period);
}

void CollectController::_loadUrdf(ros::NodeHandle & nh){
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  
  std::string urdfString;
  nh.getParam("legged_robot_description", urdfString);

  if(urdfString.empty()){
    std::string err_msg = "[CollectController] Could not load the robot description from the parameter server";
    ROS_ERROR_STREAM(err_msg);
    throw std::runtime_error(err_msg);
  }

  bool flag = urdfModel_->initString(urdfString);
  if(!flag){
    std::string err_msg = "[CollectController] Could not init urdf model by the robot description from the parameter server";
    ROS_ERROR_STREAM(err_msg);
    throw std::runtime_error(err_msg);
  }

  // get the joint limits for each joint
  for(const auto & jntName : jointNames_){
    auto jnt = urdfModel_->getJoint(jntName);
    if(jnt == nullptr){
      std::string err_msg = "[CollectController] Could not find the joint: " + jntName + " in the urdf model";
      ROS_ERROR_STREAM(err_msg);
      throw std::runtime_error(err_msg);
    }
    if(jnt->type != urdf::Joint::REVOLUTE && jnt->type != urdf::Joint::CONTINUOUS){
      std::string err_msg = "[CollectController] The joint: " + jntName + " is not a revolute or continuous joint";
      ROS_ERROR_STREAM(err_msg);
      throw std::runtime_error(err_msg);
    }
    jointLowLimits_.push_back(jnt->limits->lower);
    jointUpLimits_.push_back(jnt->limits->upper);

    ROS_INFO_STREAM("[CollectController] Joint: " << jntName << " LowerLimit: " << jnt->limits->lower << " UpperLimit: " << jnt->limits->upper);
  }
  
}


}   // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::CollectController, controller_interface::ControllerBase);
