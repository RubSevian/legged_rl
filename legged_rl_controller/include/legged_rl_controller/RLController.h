/**
 * @file RLController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <legged_rl_controller/LeggedBaseController.h>
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>

#include <torch/script.h>


namespace legged{
namespace rl{



class RLController : public LeggedBaseController{
public:
  
    RLController() = default;
    // ~RLController() override;
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

private:

  // ************************************************************************************************
  // * Structs
  // ************************************************************************************************

  struct RLConfig{
    int numActions;
    int numObservations;
    std::map<std::string, double> defaultJointAngles;
    std::vector<std::string> gymJointNames;
    struct {
      std::vector<double> lin_vel_x;
      std::vector<double> lin_vel_y;
      std::vector<double> ang_vel_yaw;
    } commandsRange;
    std::string controlType;
    double controlScale;
    std::map<std::string, double> stiffness;
    std::map<std::string, double> damping;
    struct {
      double linVel;
      double angVel;
      double dofPos;
      double dofVel;
    } obsScales;
    double clipActions;
    double clipObservations;
    std::string jitScriptPath;
  };

  struct ObservationTensor{
    torch::Tensor baseLinVel = torch::zeros({3});
    torch::Tensor baseAngVel = torch::zeros({3});
    torch::Tensor projGravity = torch::zeros({3});
    torch::Tensor commands = torch::zeros({3});
    torch::Tensor dofPos = torch::zeros({12});
    torch::Tensor dofVel = torch::zeros({12});
    torch::Tensor actions = torch::zeros({12});
  };

  void _loadUrdf(ros::NodeHandle & nh);
  std::shared_ptr<urdf::Model> urdfModel_; 

  // ************************************************************************************************
  // * Command
  // ************************************************************************************************

  ros::Subscriber cmdSub_;
  void _cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  double command_[3] = {0.0, 0.0, 0.0};  // vx, vy, omegaz

  // ************************************************************************************************
  // * Reinforcement Learning
  // ************************************************************************************************

  RLConfig rlConfig_;
  std::shared_ptr<torch::jit::script::Module> module_;
  torch::Tensor actionTensor_;
  torch::Tensor obsTensor_;
  torch::Tensor obsTensorBuf_;
  ObservationTensor obsTensorStruct_;
  void _initTensor();

  // ************************************************************************************************
  // * Observations
  // ************************************************************************************************

  std::vector<std::string> observationNames_; // its order matters, should align with the order in legged gym env
  int oneStepObsSize_;
  int obsBufSize_;
  void _updateObservation() override;

  // ************************************************************************************************
  // * Joints
  // ************************************************************************************************

  // the order of joints is different in robot.yaml and rl_cfg.yaml
  // order in robot.yaml aligns with that in unitree sdk2 or your robot
  // order in rl_cfg.yaml aligns with that in gym
  std::vector<int> jntMapRobot2Gym_;  
  std::vector<int> jntMapGym2Robot_;
  std::vector<double> jointDefaultPos_;
  std::vector<double> jointKp_;
  std::vector<double> jointKd_;
  std::vector<double> jointTorqueLimits_;
  void _initJoints(hardware_interface::RobotHW* robot_hw) override;

  // ************************************************************************************************
  // * Loop
  // ************************************************************************************************

  /**
   * @brief Check if this update can run the loop
   * 
   * @param period 
   * @return true 
   * @return false 
   */
  bool _beforeUpdate(const ros::Duration& period) override;

  /**
   * @brief set joint command and update the last command
   * @param period 
   */
  void _afterUpdate(const ros::Duration& period) override;

  // FixedSizeQueue<at::Tensor> obsQueue_;

};

} // namespace rl
} // namespace legged