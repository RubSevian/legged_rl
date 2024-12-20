/**
 * @file StandController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <legged_rl_controller/StandController.h>

#include <pluginlib/class_list_macros.hpp>


namespace legged{

bool StandController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {

  ros::NodeHandle nh;
  ros::NodeHandle nhConfig("robot_config");

  std::string imuHandleName;
  int error = 0;
  error += static_cast<int>(!nh.getParam("rl_config/init_state/default_joint_angles", defaultJointAnglesMap_));
  error += static_cast<int>(!nhConfig.getParam("imu/handle_name", imuHandleName));
  if(error > 0){
    std::string error_msg = "[StandController] Fail to load parameters";
    ROS_ERROR_STREAM(error_msg);
    throw std::runtime_error(error_msg);
  }

  auto * jointActuatorInterface = robot_hw->get<JointActuatorInterface>();
  for (const auto & [key, value] : defaultJointAnglesMap_){
    JointActuatorHandle jntHandle = jointActuatorInterface->getHandle(key);
    jointActuatorHandles_.push_back(jntHandle);
    defaultJointAngles_.push_back(value);
  }

  auto * imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuInterface->getHandle(imuHandleName);

  return true;
}

void StandController::starting(const ros::Time& time){
  startTime_ = time;
}


void StandController::stopping(const ros::Time& time){

}

void StandController::update(const ros::Time& time, const ros::Duration& period){
  auto elapsed_time = time - startTime_;

  // // 除以10，计算余数
  // int n = static_cast<int>(elapsed_time.toSec() / 10) ;
  // int remainder = elapsed_time.toSec() - n * 10;

  double Kp = 30.0;
  double Kd = 5.0;

  for(size_t i=0; i<jointNum_; i++){
      jointActuatorHandles_[i].setCommand(
        defaultJointAngles_[i], 0, Kp, Kd, 0
      );
  }

  // if(remainder < 5){
  //   for(int i = 0; i < jointNum_; i++){
  //     jointActuatorHandles_[i].setCommand(
  //       targetPos1_[i], 0, Kp, Kd, 0
  //     );
  //   }
  // }else {
  //   for(int i = 0; i < jointNum_; i++){
  //     jointActuatorHandles_[i].setCommand(
  //       targetPos2_[i], 0, Kp, Kd, 0
  //     );
  //   }
  // }
}


} // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::StandController, controller_interface::ControllerBase);