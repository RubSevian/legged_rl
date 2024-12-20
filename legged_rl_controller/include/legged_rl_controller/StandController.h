/**
 * @file StandController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_hw/hardware_interfacce/JointActuatorInterface.h>

namespace legged{

class StandController : public controller_interface::MultiInterfaceController<JointActuatorInterface, hardware_interface::ImuSensorInterface>{

public:

  StandController() = default;
  // ~StandController() override;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:

  std::vector<JointActuatorHandle> jointActuatorHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  std::map<std::string, double> defaultJointAnglesMap_;
  std::vector<double> defaultJointAngles_;
  const int jointNum_ = 12;

  ros::Time startTime_;

};


} // namespace legged