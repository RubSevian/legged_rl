#include <ros/ros.h>
#include <torch/script.h>

#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  tf2::Quaternion quat(-2.2275196948696605e-06, -0.05197961205690874, -2.359398429871434e-05, 0.998648145929673);
  auto quatInv = quat.inverse();

  tf2::Vector3 gravity = tf2::Vector3(0, 0, -1);
  auto gravityBody = tf2::quatRotate(quatInv, gravity);

  std::cout << "gravityBody: " << gravityBody.getX() << " " << gravityBody.getY() << " " << gravityBody.getZ() << std::endl;


  return 0;
}