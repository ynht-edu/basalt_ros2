#include "basalt_ros/basalt_vio.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<basalt_ros::BasaltVio>());
  rclcpp::shutdown();
  return 0;
}
