/**
 * @file RLController.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "legged_rl_controller/RLController.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace legged{
namespace rl{

bool RLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {

  ros::NodeHandle nh;
  ros::NodeHandle nhConfig("robot_config");

  odomGTSub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, &RLController::odomGTCallback, this);
  cmdSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &RLController::cmdCallback, this);

  // load the robot config parameters

  std::string imuHandleName;
  int error = 0;
  error += static_cast<int>(!nhConfig.getParam("joint_names", jointNames_));
  error += static_cast<int>(!nhConfig.getParam("imu/handle_name", imuHandleName));
  if(error > 0){
    std::string error_msg = "[RLController] Fail to load robot config parameters";
    ROS_ERROR_STREAM(error_msg);
    throw std::runtime_error(error_msg);
  }
  jointNum_ = jointNames_.size();

  // ROS Control interface initialization

  auto * jointActuatorInterface = robot_hw->get<JointActuatorInterface>();
  for(const auto & jntName : jointNames_){
    JointActuatorHandle jntHandle = jointActuatorInterface->getHandle(jntName);
    jointActuatorHandles_.push_back(jntHandle);
  }
  auto * imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuInterface->getHandle(imuHandleName);

  // Load the RL config parameters

  error = 0;
  error += static_cast<int>(!nh.getParam("env/num_actions", rlConfig_.numActions));
  error += static_cast<int>(!nh.getParam("env/num_observations", rlConfig_.numObservations));
  error += static_cast<int>(!nh.getParam("init_state/default_joint_angles", rlConfig_.defaultJointAngles));
  error += static_cast<int>(!nh.getParam("control/control_type", rlConfig_.controlType));
  error += static_cast<int>(!nh.getParam("control/action_scale", rlConfig_.controlScale));
  error += static_cast<int>(!nh.getParam("control/stiffness", rlConfig_.stiffness));
  error += static_cast<int>(!nh.getParam("control/damping", rlConfig_.damping));
  error += static_cast<int>(!nh.getParam("normalization/obs_scales/lin_vel", rlConfig_.obsScales.linVel));
  error += static_cast<int>(!nh.getParam("normalization/obs_scales/ang_vel", rlConfig_.obsScales.angVel));
  error += static_cast<int>(!nh.getParam("normalization/obs_scales/dof_pos", rlConfig_.obsScales.dofPos));
  error += static_cast<int>(!nh.getParam("normalization/obs_scales/dof_vel", rlConfig_.obsScales.dofVel));
  error += static_cast<int>(!nh.getParam("normalization/clip_actions", rlConfig_.clipActions));
  error += static_cast<int>(!nh.getParam("normalization/clip_observations", rlConfig_.clipObservations));
  error += static_cast<int>(!nh.getParam("jit_script_path", rlConfig_.jitScriptPath));
  if (error > 0) {
    std::string error_message = "[RLController] Could not retrieve one of the required parameters. Make sure you have exported the yaml files from legged gym";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // check joint number in default_joint_angles
  if(rlConfig_.defaultJointAngles.size() != jointNum_){
    std::string error_message = "[RLController] The number of joints in default_joint_angles does not match the number of joints in the robot config";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // load jit script
  try {
    ROS_INFO_STREAM("[RLController] Loading the jit script: " << rlConfig_.jitScriptPath);
    module_ = std::make_shared<torch::jit::script::Module>(torch::jit::load(rlConfig_.jitScriptPath));
  } catch (const c10::Error& e){
    std::string error_message = "[RLController] Error loading the jit script: " + rlConfig_.jitScriptPath;
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  initJoint();
  initTensor();

  // debug
  updateObservation();
  ROS_INFO_STREAM("[RLController] linVel: " << observation_.linVel );
  ROS_INFO_STREAM("[RLController] angVel: " << observation_.angVel );
  ROS_INFO_STREAM("[RLController] gravityVec: " << observation_.gravityVec );
  ROS_INFO_STREAM("[RLController] commands: " << observation_.commands );
  ROS_INFO_STREAM("[RLController] dofPos: " << observation_.dofPos );
  ROS_INFO_STREAM("[RLController] dofVel: " << observation_.dofVel );
  ROS_INFO_STREAM("[RLController] actions: " << observation_.actions );

  // obsQueue_.push(obs_);
  // obsQueue_.push(obs_);
  // obsQueue_.push(obs_);
  // obsQueue_.push(obs_);
  // obsQueue_.push(obs_);
  // obsQueue_.push(obs_);

  return true;
}

void RLController::starting(const ros::Time& time){

}

void RLController::stopping(const ros::Time& time){

}

void RLController::update(const ros::Time& time, const ros::Duration& period){
  updateObservation();
  // obsQueue_.push(obs_);

  // debug
  // check nan in obs
  if(torch::any(torch::isnan(obs_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Observation contains nan");
    std::runtime_error("Observation contains nan");
    exit(1);
  }

  // // 倒序遍历obsQueue_, 并将所有元素拼接成一个tensor作为model的输入
  // std::vector<torch::Tensor> obs_list;
  // for(int i = obsQueue_.size() - 1; i >= 0; i--){
  //   obs_list.push_back(obsQueue_[i]);
  // }
  // auto obsHistory = torch::cat(obs_list, 1);

  // set inference mode
  module_->eval();
  auto out = module_->forward({obs_}).toTensor();
  action_ = torch::clamp(out, -rlConfig_.clipActions, rlConfig_.clipActions).view({-1});

  // debug check nan
  if(torch::any(torch::isnan(action_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Action contains nan");
    std::runtime_error("Action contains nan");
    exit(1);
  }

  auto actionScaled = action_ * rlConfig_.controlScale;
  for(int i = 0; i < jointNum_; i++){
    double target = jointDefaultPos_[i] + actionScaled[i].item<float>();
    jointActuatorHandles_[i].setCommand(
      target, 0, jointKp_[i], jointKd_[i], 0
    );
  }

}

void RLController::initTensor(){
  observation_.linVel = torch::zeros({3});
  observation_.angVel = torch::zeros({3});
  observation_.gravityVec = torch::zeros({3});
  observation_.commands = torch::zeros({3});
  observation_.dofPos = torch::zeros({jointNum_});
  observation_.dofVel = torch::zeros({jointNum_});
  observation_.actions = torch::zeros({rlConfig_.numActions});
  action_ = torch::zeros({rlConfig_.numActions});
}

void RLController::initJoint(){
  // 遍历jointNames_，
  // 1. 将default_joint_angles整理成jointNames_的顺序
  // 2. 如果jointNames_中的joint的名称含有stiffness中的关键词，则将stiffness中的值赋给jointKp_
  // 3. 如果jointNames_中的joint的名称含有damping中的关键词，则将damping中的值赋给jointKd_
  jointDefaultPos_.resize(jointNum_);
  jointKp_.resize(jointNum_);
  jointKd_.resize(jointNum_);
  for(int i = 0; i < jointNum_; i++){
    // reorganize the default joint angles
    auto & jointName = jointNames_[i];
    jointDefaultPos_[i] = rlConfig_.defaultJointAngles[jointName];
    // assign the stiffness and damping
    jointKp_[i] = 0.0;
    jointKd_[i] = 0.0;
    for(const auto & [key, value] : rlConfig_.stiffness){
      if(jointNames_[i].find(key) != std::string::npos){
        jointKp_[i] = rlConfig_.stiffness[key];
      }
    }
    for(const auto & [key, value] : rlConfig_.damping){
      if(jointNames_[i].find(key) != std::string::npos){
        jointKd_[i] = rlConfig_.damping[key];
      }
    }
    ROS_INFO_STREAM("[RLController] Joint: " << jointName << " DefaultPos: " << jointDefaultPos_[i] << " Kp: " << jointKp_[i] << " Kd: " << jointKd_[i]);
  }
}

void RLController::updateObservation(){
  // get the command
  command_[0] *= rlConfig_.obsScales.linVel;
  command_[1] *= rlConfig_.obsScales.linVel;
  command_[2] *= rlConfig_.obsScales.angVel;
  observation_.commands = torch::tensor({command_[0], command_[1], command_[2]});

  // get joint position and velocity
  for(int i = 0; i < jointNum_; i++){
    observation_.dofPos[i] = (jointActuatorHandles_[i].getPosition() - jointDefaultPos_[i]) * rlConfig_.obsScales.dofPos;
    observation_.dofVel[i] = jointActuatorHandles_[i].getVelocity() * rlConfig_.obsScales.dofVel;
  }

  // cheat here for now
  // TODO: use real sensor data
  nav_msgs::Odometry odom = * odomGTBuffer_.readFromRT();
  // rotation from world to body
  tf2::Quaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  tf2::Quaternion quatInv = quat.inverse();

  tf2::Vector3 linVel(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z); // in world frame
  tf2::Vector3 angVel(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);  // in world frame
  tf2::Vector3 gravityVec(0, 0, -1);

  auto linVelBody = tf2::quatRotate(quatInv, linVel);
  auto angVelBody = tf2::quatRotate(quatInv, angVel);
  // auto linVelBody = linVel;
  // auto angVelBody = angVel;
  auto gravityVecBody = tf2::quatRotate(quatInv, gravityVec);

  observation_.linVel = torch::tensor({linVelBody.getX(), linVelBody.getY(), linVelBody.getZ()});
  observation_.angVel = torch::tensor({angVelBody.getX(), angVelBody.getY(), angVelBody.getZ()});
  observation_.gravityVec = torch::tensor({gravityVecBody.getX(), gravityVecBody.getY(), gravityVecBody.getZ()});

  observation_.actions = action_;

  std::vector<torch::Tensor> obs_list;
  obs_list.reserve(7);
  obs_list.push_back(observation_.linVel);
  obs_list.push_back(observation_.angVel);
  obs_list.push_back(observation_.gravityVec);
  obs_list.push_back(observation_.commands);
  obs_list.push_back(observation_.dofPos);
  obs_list.push_back(observation_.dofVel);
  obs_list.push_back(observation_.actions);
  auto obs = torch::cat(obs_list, 0);
  obs = obs.unsqueeze(0);
  obs_ = torch::clamp(obs, -rlConfig_.clipObservations, rlConfig_.clipObservations);

}

void RLController::odomGTCallback(const nav_msgs::Odometry::ConstPtr& msg){
  odomGTBuffer_.writeFromNonRT(*msg);
}

void RLController::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg){
  // cmdBuffer_.writeFromNonRT(*msg);
  command_[0] = msg->linear.x;
  command_[1] = msg->linear.y;
  command_[2] = msg->angular.z;
}

void RLController::loadUrdf(ros::NodeHandle & nh){
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  
  std::string urdfString;
  nh.getParam("robot_description", urdfString);

  if(urdfString.empty()){
    std::string err_msg = "[RLController] Could not load the robot description from the parameter server";
    ROS_ERROR_STREAM(err_msg);
    throw std::runtime_error(err_msg);
  }

  bool flag = urdfModel_->initString(urdfString);
  if(!flag){
    std::string err_msg = "[RLController] Could not init urdf model by the robot description from the parameter server";
    ROS_ERROR_STREAM(err_msg);
    throw std::runtime_error(err_msg);
  }

  // get the torque limit for each joint 
  for(const auto & jntName : jointNames_){
    auto jnt = urdfModel_->getJoint(jntName);
    if(jnt == nullptr){
      std::string err_msg = "[RLController] Could not find the joint: " + jntName + " in the urdf model";
      ROS_ERROR_STREAM(err_msg);
      throw std::runtime_error(err_msg);
    }
    if(jnt->type != urdf::Joint::REVOLUTE && jnt->type != urdf::Joint::CONTINUOUS){
      std::string err_msg = "[RLController] The joint: " + jntName + " is not a revolute or continuous joint";
      ROS_ERROR_STREAM(err_msg);
      throw std::runtime_error(err_msg);
    }
    jointTorqueLimits_.push_back(jnt->limits->effort);
  }
  
}


} // namespace rl
} // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::rl::RLController, controller_interface::ControllerBase);