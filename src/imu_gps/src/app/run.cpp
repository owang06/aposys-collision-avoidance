#include "imu_gps/fusion_node.hpp"
#include "imu_gps/player.hpp"
#include <iostream>

int main(int argc, char *argv[]) { // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and spin the node
  auto node = std::make_shared<sf::SensorFusionNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
