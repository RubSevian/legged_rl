/***********************************************************************************
 * @file CollectController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-30
 * 
 * @copyright Copyright (c) 2024
 * 
 ***********************************************************************************/

#pragma once

#include <legged_rl_controller/LeggedBaseController.h>
#include <legged_rl_controller/CosineCurve.h>
#include <urdf/model.h>

#include <legged_rl_controller/ActuatorState.h>

#include <random>

namespace legged{

class CollectController : public LeggedBaseController {

public:
  
  CollectController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:

  bool _beforeUpdate(const ros::Time& time, const ros::Duration& period) override;
  void _afterUpdate(const ros::Time& time, const ros::Duration& period) override;

  // some pre-defined target positions
  std::vector<std::vector<double>> targetPos_;
  int targetIdx_;
  int targetNum_;
  // cosine curves for each joint
  std::vector<CosineCurve> cosCurves_;

  std::random_device rd_;
  std::mt19937 gen_;
  std::normal_distribution<double> normalDist_;
  double noiseVariance_ = 0.1;
  double noiseScale_ = 0.1;

  ros::Duration trajTime_ = ros::Duration(5.0);

  double jointKp_ = 20.0;
  double jointKd_ = 0.5;

  std::vector<double> jointUpLimits_;
  std::vector<double> jointLowLimits_;

  void _loadUrdf(ros::NodeHandle & nh);
  std::shared_ptr<urdf::Model> urdfModel_; 

  ros::Publisher actuatorStatePub_;

};

} // namespace legged

