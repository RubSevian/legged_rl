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

  const float targetPos1_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                          -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  const float targetPos2_[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                          0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  const float targetPos3_[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                          -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  std::vector<std::string> jointNames_;
  const int jointNum_ = 12;

  ros::Time startTime_;

};


} // namespace legged