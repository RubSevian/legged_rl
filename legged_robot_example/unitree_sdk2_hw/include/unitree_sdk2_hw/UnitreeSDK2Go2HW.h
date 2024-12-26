/**
 * @file UnitreeSDK2Go2HW.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#pragma once

#include "legged_hw/LeggedHW.h"

#include <math.h>
#include <stdint.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;

namespace legged {

struct UnitreeJointData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData{
  std::string handle_name_;
  std::string frame_id_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};


class UnitreeSDK2Go2HW : public LeggedHW {

public:
  explicit UnitreeSDK2Go2HW() = default;
  
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  void read(const ros::Time& time, const ros::Duration& period) override;

  void write(const ros::Time& time, const ros::Duration& period) override;

  const std::string TOPIC_LOWCMD = "rt/lowcmd";
  const std::string TOPIC_LOWSTATE = "rt/lowstate";

  const double posStopF = (2.146E+9f);
  const double velStopF = (16000.0f);

private:

  bool setupJoints();

  bool setupImu();

  void initLowCmd();
  void lowStateMessageHandler(const void* message);
  int queryMotionStatus();
  std::string queryServiceName(std::string form,std::string name);

  // joint data for ros control
  std::vector<UnitreeJointData> jointData_;
  const int jointNum_ = 12;
  std::vector<std::string> jointNames_;
  std::vector<double> jointTorqueLimits_;
  // imu data for ros control
  UnitreeImuData imuData_;

  unitree_go::msg::dds_::LowCmd_ lowCmd_{};      // default init
  unitree_go::msg::dds_::LowState_ lowState_{};  // default init
  // unitree::robot::b2::MotionSwitcherClient motionSwitcherClient_;
  std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> motionSwitcherClient_;

  // publisher to Go2
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowCmdPublisher_;
  // subcriber to Go2
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowStateSubsriber_;

  // publisher for debug
  ros::Publisher jointStatePub_;
  ros::Publisher imuPub_;

  std::string imuTopicName_;
  std::string jointStateTopicName_;
  std::string baseLink_;

  uint32_t crc32_core(uint32_t* ptr, uint32_t len)
  {
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
      xbit = 1 << 31;
      data = ptr[i];
      for (unsigned int bits = 0; bits < 32; bits++)
      {
        if (CRC32 & 0x80000000)
        {
          CRC32 <<= 1;
          CRC32 ^= dwPolynomial;
        }
        else
        {
          CRC32 <<= 1;
        }

        if (data & xbit)
          CRC32 ^= dwPolynomial;
        xbit >>= 1;
      }
    }

    return CRC32;
  };


};



}  // namespace legged