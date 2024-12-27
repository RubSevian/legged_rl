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
  
  ros::NodeHandle nh;
  ros::NodeHandle nhConfig("robot_config");

  std::string imuHandleName;
  int error = 0;
  error += static_cast<int>(!nhConfig.getParam("joint_names", jointNames_));
  error += static_cast<int>(!nhConfig.getParam("imu/handle_name", imuHandleName));
  if(error > 0){
    std::string error_msg = "[StaticController] Fail to load parameters";
    ROS_ERROR_STREAM(error_msg);
    throw std::runtime_error(error_msg);
  }

  /**
   * @brief Allocate memory for std::vector
   */

  cosCurves_.resize(jointNum_);
  obs_.jointPos.resize(jointNum_);
  obs_.jointVel.resize(jointNum_);

  targetPos_.resize(3);
  targetPos_.at(0) = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                      -0.2, 1.36, -2.65, 0.2, 1.36, -2.65}; // ready
  targetPos_.at(1) = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, 0.0, 0.67, -1.3};  // stand
  targetPos_.at(2) = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                      -0.5, 1.36, -2.65, 0.5, 1.36, -2.65}; // down
  
  /**
   * @brief ROS subscribers initialization
   */
  targetIdxSub_ = nh.subscribe<std_msgs::Int8>("/target_idx", 1, &StaticController::targetIdxCallback, this);

  /**
   * @brief Initialize ROS Control interface
   */
  auto * jointActuatorInterface = robot_hw->get<JointActuatorInterface>();
  for(const auto & jntName : jointNames_){
    JointActuatorHandle jntHandle = jointActuatorInterface->getHandle(jntName);
    jointActuatorHandles_.push_back(jntHandle);
  }
  auto * imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuInterface->getHandle(imuHandleName);

  return true;
}

void StaticController::targetIdxCallback(const std_msgs::Int8::ConstPtr & msg){
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
  currentTime_ = time;
  updateObservation();
  for(size_t i=0; i<jointNum_; i++){
    cosCurves_[i].reset(
      obs_.jointPos[i],
      targetPos_[0][i],
      currentTime_.toSec(),
      (currentTime_+trajTime_).toSec());
  }
}


void StaticController::stopping(const ros::Time& time){

}

void StaticController::update(const ros::Time& time, const ros::Duration& period){
  ROS_INFO_THROTTLE(1, "[StaticController] RUNNING ...");
  
  currentTime_ = time;
  updateObservation();

  for(size_t i=0; i<jointNum_; i++){
    jointActuatorHandles_[i].setCommand(
      cosCurves_[i].getPos(currentTime_.toSec()),
      cosCurves_[i].getVel(currentTime_.toSec()),
      60.0, 
      5.0, 
      0.0
    );
  }
}

void StaticController::updateObservation(){
  for(size_t i=0; i<jointNum_; i++){
    obs_.jointPos[i] = jointActuatorHandles_[i].getPosition();
    obs_.jointVel[i] = jointActuatorHandles_[i].getVelocity();
  }
}

} // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::StaticController, controller_interface::ControllerBase);