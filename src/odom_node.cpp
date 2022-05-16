#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "odom_can/can_odometry.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ichthus::CANOdometry>(
    rclcpp::NodeOptions()));

  rclcpp::shutdown();
  return 0;
}
