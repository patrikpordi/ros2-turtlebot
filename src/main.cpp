#include <ros2-turtlebot/walking.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_walking>());
  rclcpp::shutdown();
  return 0;
}