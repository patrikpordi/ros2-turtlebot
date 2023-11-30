/**
 * @file main.cpp
 * @author Patrik Dominik Pördi (ppordi@umd.edu)
 * @brief
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <ros2-turtlebot/walking.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_walking>());
  rclcpp::shutdown();
  return 0;
}
