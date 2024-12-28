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
  ros::NodeHandle nhRobotConfig("robot_config");
  ros::NodeHandle nhRLConfig("rl_config");

  // ROS subscribers initialization
  cmdSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &RLController::_cmdCallback, this);

  // ************************************************************************************************
  // load the robot config parameters
  // ************************************************************************************************
  int error = 0;
  error += static_cast<int>(!nhRobotConfig.getParam("observations", observationNames_));
  error += static_cast<int>(!nhRobotConfig.getParam("one_step_obs_size", oneStepObsSize_));
  error += static_cast<int>(!nhRobotConfig.getParam("obs_buffer_size", obsBufSize_));
  if(error > 0){
    std::string error_msg = "[RLController] Fail to load robot config parameters: observations, one_step_obs_size, obs_buffer_size";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  // ************************************************************************************************
  // Load the RL config parameters
  // ************************************************************************************************
  error = 0;
  error += static_cast<int>(!nhRLConfig.getParam("env/num_actions", rlConfig_.numActions));
  error += static_cast<int>(!nhRLConfig.getParam("env/num_observations", rlConfig_.numObservations));
  error += static_cast<int>(!nhRLConfig.getParam("env/gym_joint_names", rlConfig_.gymJointNames));
  error += static_cast<int>(!nhRLConfig.getParam("init_state/default_joint_angles", rlConfig_.defaultJointAngles));
  error += static_cast<int>(!nhRLConfig.getParam("commands/ranges/lin_vel_x", rlConfig_.commandsRange.lin_vel_x));
  error += static_cast<int>(!nhRLConfig.getParam("commands/ranges/lin_vel_y", rlConfig_.commandsRange.lin_vel_y));
  error += static_cast<int>(!nhRLConfig.getParam("commands/ranges/ang_vel_yaw", rlConfig_.commandsRange.ang_vel_yaw));
  error += static_cast<int>(!nhRLConfig.getParam("control/control_type", rlConfig_.controlType));
  error += static_cast<int>(!nhRLConfig.getParam("control/action_scale", rlConfig_.controlScale));
  error += static_cast<int>(!nhRLConfig.getParam("control/stiffness", rlConfig_.stiffness));
  error += static_cast<int>(!nhRLConfig.getParam("control/damping", rlConfig_.damping));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/obs_scales/lin_vel", rlConfig_.obsScales.linVel));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/obs_scales/ang_vel", rlConfig_.obsScales.angVel));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/obs_scales/dof_pos", rlConfig_.obsScales.dofPos));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/obs_scales/dof_vel", rlConfig_.obsScales.dofVel));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/clip_actions", rlConfig_.clipActions));
  error += static_cast<int>(!nhRLConfig.getParam("normalization/clip_observations", rlConfig_.clipObservations));
  error += static_cast<int>(!nhRLConfig.getParam("jit_script_path", rlConfig_.jitScriptPath));
  if (error > 0) {
    std::string error_message = "[RLController] Could not retrieve one of the required parameters. Make sure you have exported the yaml files from legged gym";
    ROS_ERROR_STREAM(error_message);
    return false;
  }

  // check number of observations
  if (oneStepObsSize_*obsBufSize_ != rlConfig_.numObservations){
    std::string error_message = "[RLController] Check one_step_obs_size and obs_buffer_size in robot.yaml";
    ROS_ERROR_STREAM(error_message);
    return false;
  }

  // ************************************************************************************************
  // * initialize the base controller
  //    * load some necessary parameters
  //    * initialize joints and imu
  //    * initialize loop parameters
  // ************************************************************************************************
  if(!LeggedBaseController::init(robot_hw, controller_nh)){
    return false;
  }

  // ************************************************************************************************
  // * load rl policy and some confgiurations
  // ************************************************************************************************

  // load jit script
  try {
    ROS_INFO_STREAM("[RLController] Loading the jit script: " << rlConfig_.jitScriptPath);
    module_ = std::make_shared<torch::jit::script::Module>(torch::jit::load(rlConfig_.jitScriptPath));
  } catch (const c10::Error& e){
    std::string error_message = "[RLController] Error loading the jit script: " + rlConfig_.jitScriptPath;
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // initialize tensors, allocate memory
  _initTensor();

  // debug
  _updateObservation();
  // ROS_INFO_STREAM("[RLController] baseLinVel: \n" << obsTensorStruct_.baseLinVel );
  // ROS_INFO_STREAM("[RLController] baseAngVel: \n" << obsTensorStruct_.baseAngVel );
  // ROS_INFO_STREAM("[RLController] projGravity: \n" << obsTensorStruct_.projGravity );
  // ROS_INFO_STREAM("[RLController] commands: \n" << obsTensorStruct_.commands );
  // ROS_INFO_STREAM("[RLController] dofPos: \n" << obsTensorStruct_.dofPos );
  // ROS_INFO_STREAM("[RLController] dofVel: \n" << obsTensorStruct_.dofVel );
  // ROS_INFO_STREAM("[RLController] actions: \n" << obsTensorStruct_.actions );

  return true;
}

void RLController::starting(const ros::Time& time){
  LeggedBaseController::starting(time);
}

void RLController::stopping(const ros::Time& time){
  LeggedBaseController::stopping(time);
}

bool RLController::_beforeUpdate(const ros::Duration& period){
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
        jointKp_[i], 
        jointKd_[i],
        0.0
      );
    }
    return false;
  }
}

void RLController::_afterUpdate(const ros::Duration& period){

  auto actionScaled = actionTensor_ * rlConfig_.controlScale;

  for(int i = 0; i < jointNum_; i++){
    // index mapping
    int jntIdxGym = jntMapRobot2Gym_[i];
    // get the target position
    double target = jointDefaultPos_[i] + actionScaled[jntIdxGym].item<float>();
    // set the command
    jointActuatorHandles_[i].setCommand(
      target, 0, jointKp_[i], jointKd_[i], 0
    );
    // record the last command
    obs_.lastJointPosDes[i] = target;
    obs_.lastJointVelDes[i] = 0;
  }

}

void RLController::update(const ros::Time& time, const ros::Duration& period){
  _beforeUpdate(period);

  // debug
  // check nan in obs
  if(torch::any(torch::isnan(obsTensor_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Observation contains nan");
    std::runtime_error("Observation contains nan");
    exit(1);
  }

  // set inference mode
  module_->eval();
  auto out = module_->forward({obsTensorBuf_}).toTensor();
  actionTensor_ = torch::clamp(out, -rlConfig_.clipActions, rlConfig_.clipActions).view({-1});

  // debug check nan
  if(torch::any(torch::isnan(actionTensor_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Action contains nan");
    std::runtime_error("Action contains nan");
    exit(1);
  }


  _afterUpdate(period);
}

void RLController::_initTensor(){
  obsTensorStruct_.baseLinVel = torch::zeros({3});
  obsTensorStruct_.baseAngVel = torch::zeros({3});
  obsTensorStruct_.projGravity = torch::zeros({3});
  obsTensorStruct_.commands = torch::zeros({3});
  obsTensorStruct_.dofPos = torch::zeros({jointNum_});
  obsTensorStruct_.dofVel = torch::zeros({jointNum_});
  obsTensorStruct_.actions = torch::zeros({rlConfig_.numActions});
  actionTensor_ = torch::zeros({rlConfig_.numActions});
  obsTensor_ = torch::zeros({oneStepObsSize_});
  obsTensorBuf_ = torch::zeros({1, obsBufSize_*oneStepObsSize_});
}

void RLController::_initJoints(hardware_interface::RobotHW* robot_hw){
  
  // call the base controller's _initJoints
  LeggedBaseController::_initJoints(robot_hw);

  // check joint number in default_joint_angles
  if(rlConfig_.defaultJointAngles.size() != jointNum_){
    std::string error_message = "[RLController] The number of joints in default_joint_angles is not equal to the number of joints in robot.yaml";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // ************************************************************************************************
  // * Iterate jointNames_
  // * 1. align the jointDefaultPos_ with jointNames_
  // * 2. assign the stiffness and damping
  // ************************************************************************************************
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
    // debug
    ROS_INFO_STREAM("[RLController] Joint: " << jointName << " DefaultPos: " << jointDefaultPos_[i] << " Kp: " << jointKp_[i] << " Kd: " << jointKd_[i]);
  }

  // ************************************************************************************************
  // * Joint mapping from gym to robot, and from robot to gym
  // ************************************************************************************************

  jntMapGym2Robot_.resize(jointNum_, -1);
  jntMapRobot2Gym_.resize(jointNum_, -1);
  for(int i = 0; i < jointNum_; i++){
    for(int j = 0; j < jointNum_; j++){
      if(jointNames_[i] == rlConfig_.gymJointNames[j]){
        jntMapRobot2Gym_[i] = j;
        jntMapGym2Robot_[j] = i;
        break;
      }
    }
  }

  // // debug
  // ROS_INFO_STREAM("[RLController] Joint mapping: Robot Joint <-----> Gym Joint");
  // for(int i = 0; i < jointNum_; i++){
  //   ROS_INFO_STREAM("[RLController] " << jointNames_[i] << " <-----> " << rlConfig_.gymJointNames[jntMapRobot2Gym_[i]]);
  // }
  
  // check if there is any joint that is not matched
  for(int i=0; i<jointNum_; i++){
    if(jntMapGym2Robot_[i] < 0 || jntMapRobot2Gym_[i] < 0){
      std::string error_message = "[RLController] The joint: " + jointNames_[i] + " or " + rlConfig_.gymJointNames[i] + " is not matched";
      ROS_ERROR_STREAM(error_message);
      throw std::runtime_error(error_message);
    }
  }

}

void RLController::_updateObservation(){
  // get the command
  obsTensorStruct_.commands = torch::tensor({command_[0], command_[1], command_[2]});

  // get joint position and velocity
  for(int i = 0; i < jointNum_; i++){
    int jntIdxGym = jntMapRobot2Gym_[i];
    obsTensorStruct_.dofPos[jntIdxGym] = (jointActuatorHandles_[i].getPosition() - jointDefaultPos_[i]) * rlConfig_.obsScales.dofPos;
    obsTensorStruct_.dofVel[jntIdxGym] = jointActuatorHandles_[i].getVelocity() * rlConfig_.obsScales.dofVel;
  }

  // get imu data
  // quaternion
  tf2::Quaternion quat(
    imuSensorHandle_.getOrientation()[0], // x
    imuSensorHandle_.getOrientation()[1], // y
    imuSensorHandle_.getOrientation()[2], // z
    imuSensorHandle_.getOrientation()[3]  // w
  );
  tf2::Quaternion quatInv = quat.inverse();
  // angular velocity
  obsTensorStruct_.baseAngVel = torch::tensor({imuSensorHandle_.getAngularVelocity()[0], 
                                        imuSensorHandle_.getAngularVelocity()[1], 
                                        imuSensorHandle_.getAngularVelocity()[2]});
  
  // project the gravity vector to the body frame
  // i.e. express the gravity vector in the body frame
  tf2::Vector3 gravityVec(0, 0, -1);
  auto projGravity = tf2::quatRotate(quatInv, gravityVec);
  obsTensorStruct_.projGravity = torch::tensor({projGravity.getX(), projGravity.getY(), projGravity.getZ()});

  // last action
  obsTensorStruct_.actions = actionTensor_;

  std::vector<torch::Tensor> obs_list;
  obs_list.reserve(observationNames_.size());
  // push back the observations according to the order in observationNames_
  for(const auto & obsName : observationNames_){
    if(obsName == "commands"){  // 3
      obs_list.push_back(obsTensorStruct_.commands);
    } else if(obsName == "base_ang_vel"){ // 3
      obs_list.push_back(obsTensorStruct_.baseAngVel);
    } else if(obsName == "projected_gravity"){  // 3
      obs_list.push_back(obsTensorStruct_.projGravity);
    } else if(obsName == "dof_pos"){  // 12
      obs_list.push_back(obsTensorStruct_.dofPos);
    } else if(obsName == "dof_vel"){  // 12
      obs_list.push_back(obsTensorStruct_.dofVel);
    } else if(obsName == "actions"){  // 12
      obs_list.push_back(obsTensorStruct_.actions);
    } else {
      std::string error_message = "[RLController] The observation: " + obsName + " is not supported";
      ROS_ERROR_STREAM(error_message);
      throw std::runtime_error(error_message);
    }
  }

  auto obs = torch::cat(obs_list, 0);
  obs = obs.unsqueeze(0);
  obsTensor_ = torch::clamp(obs, -rlConfig_.clipObservations, rlConfig_.clipObservations);

  // update the observation buffer
  obsTensorBuf_ = torch::cat({obsTensor_, obsTensorBuf_.slice(1, 0, obsBufSize_*oneStepObsSize_-oneStepObsSize_)}, 1);
}

void RLController::_cmdCallback(const geometry_msgs::Twist::ConstPtr& msg){
  // clip the command
  command_[0] = std::clamp(msg->linear.x, rlConfig_.commandsRange.lin_vel_x[0], rlConfig_.commandsRange.lin_vel_x[1]);
  command_[1] = std::clamp(msg->linear.y, rlConfig_.commandsRange.lin_vel_y[0], rlConfig_.commandsRange.lin_vel_y[1]);
  command_[2] = std::clamp(msg->angular.z, rlConfig_.commandsRange.ang_vel_yaw[0], rlConfig_.commandsRange.ang_vel_yaw[1]);

  // scale the command
  command_[0] = command_[0] * rlConfig_.obsScales.linVel;
  command_[1] = command_[1] * rlConfig_.obsScales.linVel;
  command_[2] = command_[2] * rlConfig_.obsScales.angVel;
}

void RLController::_loadUrdf(ros::NodeHandle & nh){
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