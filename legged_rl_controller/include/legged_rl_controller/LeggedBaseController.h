/**
 * @file LeggedBaseController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_hw/hardware_interfacce/JointActuatorInterface.h>

namespace legged{

class LeggedBaseController : public controller_interface::MultiInterfaceController<JointActuatorInterface, hardware_interface::ImuSensorInterface>{

public:

  LeggedBaseController() = default;

  /**
   * @brief init controller
   * 
   * In this method resources from different interfaces are claimed, and other
   * non real-time initialization is performed, such as setup of ROS interfaces
   * and resource pre-allocation.
   * 
   * @param robot_hw Robot hardware abstraction containing a subset of the entire robot.
   * @param nh A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   * @return true 
   * @return false 
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

protected:

  // ************************************************************************************************
  // * Observation
  // ************************************************************************************************
  struct Observation{
    std::vector<double> jointPos;
    std::vector<double> jointVel;
    double imuOri[4]; // quaternion, x, y, z, w
    double imuAngVel[3];  // angular velocity
    double imuLinAcc[3];  // linear acceleration
    std::vector<double> lastJointPosDes;
    std::vector<double> lastJointVelDes;
  };

  Observation obs_;

  /**
   * @brief update observation, including joint info, imu info, etc.
   *  Must be called after _initJoints
   */
  virtual void _updateObservation();

  /**
   * @brief Init observation structure
   */
  virtual void _initObservation();

  // ************************************************************************************************
  // * Joints and IMU
  // ************************************************************************************************
  std::vector<std::string> jointNames_;
  int jointNum_;
  std::vector<JointActuatorHandle> jointActuatorHandles_;
  /**
   * @brief Initialize joints configuration, get joint handles from ros control interface
   * 
   * @param robot_hw 
   */
  virtual void _initJoints(hardware_interface::RobotHW* robot_hw);

  // ************************************************************************************************
  // * IMU
  // ************************************************************************************************
  std::string imuHandleName_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;
  /**
   * @brief Initialize IMU configuration, get imu handle from ros control interface
   * 
   * @param robot_hw 
   */
  virtual void _initImu(hardware_interface::RobotHW* robot_hw);

  // ************************************************************************************************
  // * Loop parameters
  // * ref: https://github.com/ros-controls/ros_control/pull/127
  // ************************************************************************************************
  double loopFrequency_;  // loop frequency of the controller, it differs from the loop frequency of the controller manager
  double cmFrequency_;  // loop frequency of the controller manager
  int loopEveryN_;
  int loopCounter_ = 1;
  double timePassed_ = 0.0;   // time passed since last loop

  /**
   * @brief Check if this loop should run, so that the controller runs at the desired frequency
   *  Calculate the time passed since the last loop
   *  Remember to set the last command if the loop is not run
   *  Remember to record the last command
   * @return true 
   * @return false 
   */
  bool _runThisLoop(double period, bool debug=false);

  virtual bool _beforeUpdate(const ros::Duration& period) = 0;
  virtual void _afterUpdate(const ros::Duration& period) = 0;

};


} // namespace legged