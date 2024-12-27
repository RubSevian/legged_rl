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

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_hw/hardware_interfacce/JointActuatorInterface.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <torch/script.h>

#include <urdf/model.h>

// #include <queue>
// #include <deque>

// template <typename T>
// class FixedSizeQueue {
// public:
//     FixedSizeQueue(size_t maxSize) : maxSize_(maxSize) {}

//     void push(const T& value) {
//         if (queue_.size() >= maxSize_) {
//             queue_.pop_front();
//         }
//         queue_.push_back(value);
//     }

//     void pop() {
//         if (!queue_.empty()) {
//             queue_.pop_front();
//         }
//     }

//     T& front() {
//         return queue_.front();
//     }

//     const T& front() const {
//         return queue_.front();
//     }

//     T& back() {
//         return queue_.back();
//     }

//     const T& back() const {
//         return queue_.back();
//     }

//     size_t size() const {
//         return queue_.size();
//     }

//     bool empty() const {
//         return queue_.empty();
//     }

//     // 按照idx访问元素
//     T& operator[](size_t idx) {
//         // check if idx is out of range
//         if (idx >= queue_.size()) {
//             throw std::out_of_range("Index out of range");
//         }
//         return queue_[idx];
//     }

// private:
//     std::deque<T> queue_;
//     size_t maxSize_;
// };

namespace legged{
namespace rl{

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

class RLController : public controller_interface::MultiInterfaceController<JointActuatorInterface, hardware_interface::ImuSensorInterface>{
public:
  
    RLController() = default;
    // ~RLController() override;
  
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

private:

  void loadUrdf(ros::NodeHandle & nh);
  std::shared_ptr<urdf::Model> urdfModel_; 

  std::vector<JointActuatorHandle> jointActuatorHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  ros::Subscriber cmdSub_;
  // realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmdBuffer_;
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  double command_[3] = {0.0, 0.0, 0.0};  // vx, vy, omegaz

  RLConfig rlConfig_;
  std::shared_ptr<torch::jit::script::Module> module_;
  torch::Tensor action_;
  torch::Tensor obs_;
  torch::Tensor obsBuf_;
  ObservationTensor observation_;
  void initTensor();

  std::vector<std::string> observationNames_; // its order matters, should align with the order in legged gym env
  int oneStepObsSize_;
  int obsBufSize_;
  void updateObservation();

  std::vector<std::string> jointNames_;
  // the order of joints is different in robot.yaml and rl_cfg.yaml
  // order in robot.yaml aligns with that in unitree sdk2 or your robot
  // order in rl_cfg.yaml aligns with that in gym
  std::vector<int> jntMapRobot2Gym_;  
  std::vector<int> jntMapGym2Robot_;
  int jointNum_;
  std::vector<double> jointDefaultPos_;
  std::vector<double> jointKp_;
  std::vector<double> jointKd_;
  std::vector<double> jointTorqueLimits_;
  void initJoint();




  // FixedSizeQueue<at::Tensor> obsQueue_;

};

} // namespace rl
} // namespace legged