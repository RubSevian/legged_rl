/**
 * @file StaticController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_hw/hardware_interfacce/JointActuatorInterface.h>
#include <legged_rl_controller/CosineCurve.h>
#include <std_msgs/Int8.h>

namespace legged{

class StaticController : public controller_interface::MultiInterfaceController<JointActuatorInterface, hardware_interface::ImuSensorInterface>{

public:

  StaticController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:

  struct Observation{
    std::vector<double> jointPos;
    std::vector<double> jointVel;
  };

  ros::Subscriber targetIdxSub_;
  void targetIdxCallback(const std_msgs::Int8::ConstPtr & msg);
  int targetIdx_ = 0;

  std::vector<JointActuatorHandle> jointActuatorHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  std::vector<CosineCurve> cosCurves_;
  std::vector<std::string> jointNames_;
  const int jointNum_ = 12;

  // double targetPos1_[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
  //                           -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  // double targetPos2_[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
  //                           0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  // double targetPos3_[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
  //                           -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  std::vector<std::vector<double>> targetPos_;


  Observation obs_;
  void updateObservation();

  ros::Time currentTime_;
  ros::Duration trajTime_ = ros::Duration(5.0);

};


} // namespace legged