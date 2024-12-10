/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file LeggedHW.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <legged_hw/hardware_interfacce/JointActuatorInterface.h>

namespace legged
{

class LeggedHW : public hardware_interface::RobotHW{

public:
  LeggedHW() = default;

  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

protected:

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
  hardware_interface::ImuSensorInterface imuSensorInterface_;    // NOLINT(misc-non-private-member-variables-in-classes)
  JointActuatorInterface jointActuatorInterface_;                    // NOLINT(misc-non-private-member-variables-in-classes)
  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param rootNh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& rootNh);

private:


};

} // namespace legged