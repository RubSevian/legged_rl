/**
 * @file UnitreeSDK2Go2HW.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "unitree_sdk2_hw/UnitreeSDK2Go2HW.h"

#include <sensor_msgs/Imu.h>

namespace legged{

bool UnitreeSDK2Go2HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  ros::NodeHandle nhP("~");
  ros::NodeHandle nhConfig("robot_config");
  int error = 0;
  // FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2
  // FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
  // RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
  // RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11
  jointNames_.reserve(jointNum_);
  error += static_cast<int>(!nhConfig.getParam("joint_names", jointNames_));
  std::string imuTopicName;
  error += static_cast<int>(!nhConfig.getParam("imu/topic_name", imuTopicName));
  error += static_cast<int>(!nhConfig.getParam("imu/handle_name", imuData_.handle_name_));
  error += static_cast<int>(!nhConfig.getParam("imu/frame_id", imuData_.frame_id_));
  std::string networkInterface;
  error += static_cast<int>(!nhConfig.getParam("unitree_sdk2/network_interface", networkInterface));

  if(error > 0){
    std::string err_msg = "could not retrieve one of the required parameters: joint_names or imu/topic_name or imu/handle_name or imu/frame_id";
    ROS_ERROR_STREAM(err_msg);
    throw std::runtime_error(err_msg);
  }
  
  imuPub_ = nhP.advertise<sensor_msgs::Imu>(imuTopicName, 1);

  /**
   * @brief Initialize ROS Control
   * 
   */
  setupJoints();
  setupImu();

  // load urdf
  if(!loadUrdf(root_nh)){
    std::string err_msg = "[LeggedMujocoHW] Could not load the urdf model";
    ROS_ERROR_STREAM(err_msg);
    return false;
  }

  // parse torque limit for each joint
  for(const auto & jntName : jointNames_){
    auto jnt = urdfModel_->getJoint(jntName);
    if(jnt == nullptr){
      std::string err_msg = "[LeggedMujocoHW] Could not find the joint: " + jntName + " in the urdf model";
      ROS_ERROR_STREAM(err_msg);
      return false;
    }
    jointTorqueLimits_.push_back(jnt->limits->effort);
  }

  /**
   * @brief Initialize Unitree SDK2
   */
  ChannelFactory::Instance()->Init(0, networkInterface);
  initLowCmd();
  // create publisher to go2
  lowCmdPublisher_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
  lowCmdPublisher_->InitChannel();
  // create subscriber to go2
  lowStateSubsriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
  lowStateSubsriber_->InitChannel(std::bind(&UnitreeSDK2Go2HW::lowStateMessageHandler, this, std::placeholders::_1), 1);

  /**
   * @brief Init motion switcher client
   */

  if(networkInterface != "lo"){
    // if not simulation, switch to motion
    motionSwitcherClient_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    motionSwitcherClient_->SetTimeout(3.0f);
    motionSwitcherClient_->Init();
    // shutdown motion-control-related service
    while(queryMotionStatus()){
      ROS_INFO("[UnitreeSDK2Go2HW] Shutting down the motion control-related service...");
      int32_t ret = motionSwitcherClient_->ReleaseMode();
      if (ret == 0){
        ROS_INFO("[UnitreeSDK2Go2HW] ReleaseMode succeeded.");
      } else {
        ROS_ERROR_STREAM("[UnitreeSDK2Go2HW] ReleaseMode failed. Error code: " << ret);
      }
      sleep(5);
    }
  }



  return true;
}

void UnitreeSDK2Go2HW::read(const ros::Time& time, const ros::Duration& period) {
  // read joint sensor data
  for (size_t i=0; i<jointNum_; i++){
    jointData_[i].pos_ = lowState_.motor_state()[i].q();
    jointData_[i].vel_ = lowState_.motor_state()[i].dq();
    jointData_[i].tau_ = lowState_.motor_state()[i].tau_est();
  }

  // read imu sensor data
  // imuData_.ori_ convention: x, y, z, w
  // unitree sdk2 quaternion: w, x, y, z
  imuData_.ori_[0] = lowState_.imu_state().quaternion()[1]; // x
  imuData_.ori_[1] = lowState_.imu_state().quaternion()[2]; // y
  imuData_.ori_[2] = lowState_.imu_state().quaternion()[3]; // z
  imuData_.ori_[3] = lowState_.imu_state().quaternion()[0]; // w

  imuData_.angularVel_[0] = lowState_.imu_state().gyroscope()[0];
  imuData_.angularVel_[1] = lowState_.imu_state().gyroscope()[1];
  imuData_.angularVel_[2] = lowState_.imu_state().gyroscope()[2];

  imuData_.linearAcc_[0] = lowState_.imu_state().accelerometer()[0];
  imuData_.linearAcc_[1] = lowState_.imu_state().accelerometer()[1];
  imuData_.linearAcc_[2] = lowState_.imu_state().accelerometer()[2];

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  for(size_t i=0; i<jointNum_; i++){
    jointData_[i].posDes_ = jointData_[i].pos_;
    jointData_[i].velDes_ = jointData_[i].vel_;
    jointData_[i].ff_ = 0.0;
    jointData_[i].kp_ = 0.0;
    jointData_[i].kd_ = 0.0;
  }

  // publish for debug
  sensor_msgs::Imu imuMsg;
  imuMsg.header.stamp = time;
  imuMsg.orientation.x = imuData_.ori_[0];
  imuMsg.orientation.y = imuData_.ori_[1];
  imuMsg.orientation.z = imuData_.ori_[2];
  imuMsg.orientation.w = imuData_.ori_[3];
  imuMsg.angular_velocity.x = imuData_.angularVel_[0];
  imuMsg.angular_velocity.y = imuData_.angularVel_[1];
  imuMsg.angular_velocity.z = imuData_.angularVel_[2];
  imuMsg.linear_acceleration.x = imuData_.linearAcc_[0];
  imuMsg.linear_acceleration.y = imuData_.linearAcc_[1];
  imuMsg.linear_acceleration.z = imuData_.linearAcc_[2];
  imuPub_.publish(imuMsg);

}

void UnitreeSDK2Go2HW::write(const ros::Time& time, const ros::Duration& period){
  // ROS_INFO_STREAM_THROTTLE(1, "[UnitreeSDK2Go2HW] Joint 0 target pos: " << jointData_[0].posDes_);

  // write joint command
  for(size_t i=0; i<jointNum_; i++){
    lowCmd_.motor_cmd()[i].mode() = (0x01); // FoC
    lowCmd_.motor_cmd()[i].q() = jointData_[i].posDes_;
    lowCmd_.motor_cmd()[i].dq() = jointData_[i].velDes_;
    lowCmd_.motor_cmd()[i].kp() = jointData_[i].kp_;
    lowCmd_.motor_cmd()[i].kd() = jointData_[i].kd_;
    lowCmd_.motor_cmd()[i].tau() = jointData_[i].ff_;

    // TODO: add torque limits
    
    // double target = jointData_[i].ff_ + 
    //                 jointData_[i].kp_ * (jointData_[i].posDes_ - jointData_[i].pos_) +
    //                 jointData_[i].kd_ * (jointData_[i].velDes_ - jointData_[i].vel_);
    // lowCmd_.motor_cmd()[i].q() = 0.0;
    // lowCmd_.motor_cmd()[i].dq() = 0.0;
    // lowCmd_.motor_cmd()[i].kp() = 0.0;
    // lowCmd_.motor_cmd()[i].kd() = 0.0;
    // lowCmd_.motor_cmd()[i].tau() = std::max(-jointTorqueLimits_[i], std::min(target, jointTorqueLimits_[i]));

  }

  lowCmd_.crc() = crc32_core((uint32_t *)&lowCmd_, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);

  lowCmdPublisher_->Write(lowCmd_);

}

bool UnitreeSDK2Go2HW::setupJoints(){
  jointData_.resize(jointNum_);
  // FR_0 -> 0 , FR_1 -> 1  , FR_2 -> 2   电机顺序，目前只用12电机，后面保留。
  // FL_0 -> 3 , FL_1 -> 4  , FL_2 -> 5
  // RR_0 -> 6 , RR_1 -> 7  , RR_2 -> 8
  // RL_0 -> 9 , RL_1 -> 10 , RL_2 -> 11
  for(size_t i=0; i<jointNum_; i++){
    const std::string & jntName = jointNames_[i];
    // register joint state handle to joint state interface
    hardware_interface::JointStateHandle jntStateHandle(jntName, &jointData_[i].pos_, &jointData_[i].vel_, &jointData_[i].tau_);
    jointStateInterface_.registerHandle(jntStateHandle);
    // register joint handle to hybrid joint interface
    JointActuatorHandle jntHandle(jntStateHandle, &jointData_[i].posDes_, &jointData_[i].velDes_, &jointData_[i].kp_, &jointData_[i].kd_, &jointData_[i].ff_);
    jointActuatorInterface_.registerHandle(jntHandle);
  }

  return true;
}

bool UnitreeSDK2Go2HW::setupImu(){

  hardware_interface::ImuSensorHandle imuSensorHandle(imuData_.handle_name_, imuData_.frame_id_,
                                                      imuData_.ori_, imuData_.oriCov_, 
                                                      imuData_.angularVel_, imuData_.angularVelCov_, 
                                                      imuData_.linearAcc_, imuData_.linearAccCov_);

  imuSensorInterface_.registerHandle(imuSensorHandle);

  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

void UnitreeSDK2Go2HW::initLowCmd(){

  lowCmd_.head()[0] = 0xFE;
  lowCmd_.head()[1] = 0xEF;
  lowCmd_.level_flag() = 0xFF;
  lowCmd_.gpio() = 0;

  for(size_t i=0; i<20; i++){
    lowCmd_.motor_cmd()[i].mode() = (0x01); // FoC
    lowCmd_.motor_cmd()[i].q() = (posStopF);
    lowCmd_.motor_cmd()[i].dq() = (velStopF);
    lowCmd_.motor_cmd()[i].kp() = (0);
    lowCmd_.motor_cmd()[i].kd() = (0);
    lowCmd_.motor_cmd()[i].tau() = (0);
  }

}

void UnitreeSDK2Go2HW::lowStateMessageHandler(const void * message){
  lowState_ = *(unitree_go::msg::dds_::LowState_*)message;
}

int UnitreeSDK2Go2HW::queryMotionStatus(){
  std::string robotForm,motionName;
  int motionStatus;
  int32_t ret = motionSwitcherClient_->CheckMode(robotForm,motionName);
  if (ret == 0){
    ROS_INFO("[UnitreeSDK2Go2HW] CheckMode succeeded.");
  } else {
    ROS_ERROR_STREAM("[UnitreeSDK2Go2HW] CheckMode failed. Error code: " << ret);
  }
  if (motionName.empty()){
    ROS_INFO("[UnitreeSDK2Go2HW] The motion control-related service is deactivated.");
    motionStatus = 0;
  } else {
    std::string serviceName = queryServiceName(robotForm,motionName);
    ROS_INFO_STREAM("[UnitreeSDK2Go2HW] The motion control-related service: " << serviceName << " is activated.");
    motionStatus = 1;
  }
  return motionStatus;
}

std::string UnitreeSDK2Go2HW::queryServiceName(std::string form,std::string name){
  if(form == "0")
  {
    if(name == "normal" ) return "sport_mode"; 
    if(name == "ai" ) return "ai_sport"; 
    if(name == "advanced" ) return "advanced_sport"; 
  }
  else
  {
    if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
    if(name == "normal-w" ) return "wheeled_sport(b2W)";
  }
  return "";
}


} // namespace legged