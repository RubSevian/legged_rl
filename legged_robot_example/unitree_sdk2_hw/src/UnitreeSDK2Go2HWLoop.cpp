/**
 * @file UnitreeSDK2Go2HWLoop.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-12-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>

#include <legged_hw/LeggedHWLoop.h>
#include <unitree_sdk2_hw/UnitreeSDK2Go2HW.h>



int main(int argc, char** argv) {
  using namespace legged;
  
  ros::init(argc, argv, "unitree_sdk2_go2");
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  // std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
  //         << "Press Enter to continue..." << std::endl;
  ROS_WARN_STREAM("WARNING: Make sure the robot is hung up or lying on the ground.");
  ROS_WARN_STREAM("Press Enter to continue...");
  std::cin.ignore();

  // We run the ROS loop in a separate thread as external calls, such
  // as service callbacks loading controllers, can block the (main) control loop

  ros::AsyncSpinner spinner(3);
  spinner.start();

  try{
    std::shared_ptr<UnitreeSDK2Go2HW> unitreeHW = std::make_shared<UnitreeSDK2Go2HW>();
    unitreeHW->init(nh, nhP);

    LeggedHWLoop loop(nh, unitreeHW);

    ros::waitForShutdown();
  } catch(const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}




