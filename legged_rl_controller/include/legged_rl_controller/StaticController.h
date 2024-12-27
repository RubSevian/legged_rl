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

#include <legged_rl_controller/LeggedBaseController.h>
#include <legged_rl_controller/CosineCurve.h>
#include <std_msgs/Int8.h>

namespace legged{

class StaticController : public LeggedBaseController{

public:

  StaticController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:

  bool _beforeUpdate(const ros::Duration& period) override;
  void _afterUpdate(const ros::Duration& period) override;

  // some pre-defined target positions
  std::vector<std::vector<double>> targetPos_;
  // cosine curves for each joint
  std::vector<CosineCurve> cosCurves_;
  // target index subscriber
  ros::Subscriber targetIdxSub_;
  void _targetIdxCallback(const std_msgs::Int8::ConstPtr & msg);
  int targetIdx_ = 0;

  ros::Time currentTime_;
  ros::Duration trajTime_ = ros::Duration(5.0);
};


} // namespace legged