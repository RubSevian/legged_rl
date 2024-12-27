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

  // load the robot config parameters

  std::string imuHandleName;
  int error = 0;
  error += static_cast<int>(!nhRobotConfig.getParam("joint_names", jointNames_));
  error += static_cast<int>(!nhRobotConfig.getParam("imu/handle_name", imuHandleName));
  error += static_cast<int>(!nhRobotConfig.getParam("observations", observationNames_));
  error += static_cast<int>(!nhRobotConfig.getParam("one_step_obs_size", oneStepObsSize_));
  error += static_cast<int>(!nhRobotConfig.getParam("obs_buffer_size", obsBufSize_));
  if(error > 0){
    std::string error_msg = "[RLController] Fail to load robot config parameters";
    ROS_ERROR_STREAM(error_msg);
    throw std::runtime_error(error_msg);
  }
  jointNum_ = jointNames_.size();

  // Load the RL config parameters

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
    throw std::runtime_error(error_message);
  }

  // check number of observations
  if (oneStepObsSize_*obsBufSize_ != rlConfig_.numObservations){
    std::string error_message = "[RLController] Check one_step_obs_size and obs_buffer_size in robot.yaml";
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  // ROS subscribers initialization
  cmdSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &RLController::cmdCallback, this);

  // ROS Control interface initialization

  auto * jointActuatorInterface = robot_hw->get<JointActuatorInterface>();
  for(const auto & jntName : jointNames_){
    JointActuatorHandle jntHandle = jointActuatorInterface->getHandle(jntName);
    jointActuatorHandles_.push_back(jntHandle);
  }
  auto * imuInterface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  imuSensorHandle_ = imuInterface->getHandle(imuHandleName);

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
  // ROS_INFO_STREAM("[RLController] baseLinVel: \n" << observation_.baseLinVel );
  // ROS_INFO_STREAM("[RLController] baseAngVel: \n" << observation_.baseAngVel );
  // ROS_INFO_STREAM("[RLController] projGravity: \n" << observation_.projGravity );
  // ROS_INFO_STREAM("[RLController] commands: \n" << observation_.commands );
  // ROS_INFO_STREAM("[RLController] dofPos: \n" << observation_.dofPos );
  // ROS_INFO_STREAM("[RLController] dofVel: \n" << observation_.dofVel );
  // ROS_INFO_STREAM("[RLController] actions: \n" << observation_.actions );

  return true;
}

void RLController::starting(const ros::Time& time){

}

void RLController::stopping(const ros::Time& time){

}

void RLController::update(const ros::Time& time, const ros::Duration& period){
  updateObservation();

  ROS_INFO_STREAM_THROTTLE(1, "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] baseAngVel: \n" << observation_.baseAngVel );
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] projGravity: \n" << observation_.projGravity );
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] commands: \n" << observation_.commands );
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] dofPos: \n" << observation_.dofPos );
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] dofVel: \n" << observation_.dofVel );
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] actions: \n" << observation_.actions );

  // debug
  // check nan in obs
  if(torch::any(torch::isnan(obs_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Observation contains nan");
    std::runtime_error("Observation contains nan");
    exit(1);
  }

  // set inference mode
  module_->eval();
  auto out = module_->forward({obsBuf_}).toTensor();
  action_ = torch::clamp(out, -rlConfig_.clipActions, rlConfig_.clipActions).view({-1});

  // debug check nan
  if(torch::any(torch::isnan(action_)).item<bool>()){
    ROS_ERROR_STREAM("[RLController] Action contains nan");
    std::runtime_error("Action contains nan");
    exit(1);
  }

  auto actionScaled = action_ * rlConfig_.controlScale;

  ROS_INFO_STREAM_THROTTLE(1, "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  ROS_INFO_STREAM_THROTTLE(1, "[RLController] actionScaled: " << actionScaled);
  ROS_INFO_STREAM_THROTTLE(1, "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

  // for(int i = 0; i < jointNum_; i++){
  //   int jntIdxGym = jntMapRobot2Gym_[i];
  //   double target = jointDefaultPos_[i] + actionScaled[jntIdxGym].item<float>();
  //   jointActuatorHandles_[i].setCommand(
  //     target, 0, jointKp_[i], jointKd_[i], 0
  //   );
  // }

}

void RLController::initTensor(){
  observation_.baseLinVel = torch::zeros({3});
  observation_.baseAngVel = torch::zeros({3});
  observation_.projGravity = torch::zeros({3});
  observation_.commands = torch::zeros({3});
  observation_.dofPos = torch::zeros({jointNum_});
  observation_.dofVel = torch::zeros({jointNum_});
  observation_.actions = torch::zeros({rlConfig_.numActions});
  action_ = torch::zeros({rlConfig_.numActions});
  obs_ = torch::zeros({oneStepObsSize_});
  obsBuf_ = torch::zeros({1, obsBufSize_*oneStepObsSize_});
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

  // init joint idx mapping
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

  // debug
  ROS_INFO_STREAM("[RLController] Joint mapping: Robot Joint <-----> Gym Joint");
  for(int i = 0; i < jointNum_; i++){
    ROS_INFO_STREAM("[RLController] " << jointNames_[i] << " <-----> " << rlConfig_.gymJointNames[jntMapRobot2Gym_[i]]);
  }
  
  // check if there is any joint that is not matched
  for(int i=0; i<jointNum_; i++){
    if(jntMapGym2Robot_[i] < 0 || jntMapRobot2Gym_[i] < 0){
      std::string error_message = "[RLController] The joint: " + jointNames_[i] + " or " + rlConfig_.gymJointNames[i] + " is not matched";
      ROS_ERROR_STREAM(error_message);
      throw std::runtime_error(error_message);
    }
  }

}

void RLController::updateObservation(){
  // get the command
  observation_.commands = torch::tensor({command_[0], command_[1], command_[2]});

  // get joint position and velocity
  for(int i = 0; i < jointNum_; i++){
    int jntIdxGym = jntMapRobot2Gym_[i];
    observation_.dofPos[jntIdxGym] = (jointActuatorHandles_[i].getPosition() - jointDefaultPos_[i]) * rlConfig_.obsScales.dofPos;
    observation_.dofVel[jntIdxGym] = jointActuatorHandles_[i].getVelocity() * rlConfig_.obsScales.dofVel;
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
  observation_.baseAngVel = torch::tensor({imuSensorHandle_.getAngularVelocity()[0], 
                                        imuSensorHandle_.getAngularVelocity()[1], 
                                        imuSensorHandle_.getAngularVelocity()[2]});
  
  // project the gravity vector to the body frame
  // i.e. express the gravity vector in the body frame
  tf2::Vector3 gravityVec(0, 0, -1);
  auto projGravity = tf2::quatRotate(quatInv, gravityVec);
  observation_.projGravity = torch::tensor({projGravity.getX(), projGravity.getY(), projGravity.getZ()});

  // last action
  observation_.actions = action_;

  std::vector<torch::Tensor> obs_list;
  obs_list.reserve(observationNames_.size());
  // push back the observations according to the order in observationNames_
  for(const auto & obsName : observationNames_){
    if(obsName == "commands"){  // 3
      obs_list.push_back(observation_.commands);
    } else if(obsName == "base_ang_vel"){ // 3
      obs_list.push_back(observation_.baseAngVel);
    } else if(obsName == "projected_gravity"){  // 3
      obs_list.push_back(observation_.projGravity);
    } else if(obsName == "dof_pos"){  // 12
      obs_list.push_back(observation_.dofPos);
    } else if(obsName == "dof_vel"){  // 12
      obs_list.push_back(observation_.dofVel);
    } else if(obsName == "actions"){  // 12
      obs_list.push_back(observation_.actions);
    } else {
      std::string error_message = "[RLController] The observation: " + obsName + " is not supported";
      ROS_ERROR_STREAM(error_message);
      throw std::runtime_error(error_message);
    }
  }

  auto obs = torch::cat(obs_list, 0);
  obs = obs.unsqueeze(0);
  obs_ = torch::clamp(obs, -rlConfig_.clipObservations, rlConfig_.clipObservations);

  // update the observation buffer
  obsBuf_ = torch::cat({obs_, obsBuf_.slice(1, 0, obsBufSize_*oneStepObsSize_-oneStepObsSize_)}, 1);
}

void RLController::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg){
  // clip the command
  command_[0] = std::clamp(msg->linear.x, rlConfig_.commandsRange.lin_vel_x[0], rlConfig_.commandsRange.lin_vel_x[1]);
  command_[1] = std::clamp(msg->linear.y, rlConfig_.commandsRange.lin_vel_y[0], rlConfig_.commandsRange.lin_vel_y[1]);
  command_[2] = std::clamp(msg->angular.z, rlConfig_.commandsRange.ang_vel_yaw[0], rlConfig_.commandsRange.ang_vel_yaw[1]);

  // scale the command
  command_[0] = command_[0] * rlConfig_.obsScales.linVel;
  command_[1] = command_[1] * rlConfig_.obsScales.linVel;
  command_[2] = command_[2] * rlConfig_.obsScales.angVel;
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