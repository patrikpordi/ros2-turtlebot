/**
 * @file walking.cpp
 * @author Patrik Dominik Pördi (ppordi@umd.edu)
 * @brief
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <random>
#include <ros2-turtlebot/walking.hpp>
/**
 * @brief Function to publish the velocity values
 *
 * @param msg
 */
void Turtle_walking::publish_velocity(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  publisher_->publish(*msg);
}
/**
 * @brief Function to act based on the laser scan data
 *
 * @param msg
 */
void Turtle_walking::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int16_t min_angle = 45.0;
  int16_t max_angle = 315.0;
  double distance = msg->range_max;
  geometry_msgs::msg::Twist cmd_vel_msg;
  for (int16_t i = 0; i < int16_t(msg->ranges.size()); i++) {
    if (i <= min_angle || i >= max_angle) {
      if (!std::isnan(msg->ranges[i])) {
        double scan_dist = msg->ranges[i];
        if (scan_dist < distance) {
          distance = scan_dist;
        }
      }
    }
  }

  if (distance <= safety_distance) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("Turtle_walking"),
                       "Houston we are getting to close!");

    cmd_vel_msg.linear.x = 0.0;
    if (rotate == -1) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"), "Turn Right!");
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"), "Turn Left!");
    }
    cmd_vel_msg.angular.z = rotate * 1.0;
  } else {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);
    rotate = (dis(gen) == 0) ? -1 : 1;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"), "Move Forward!");
    cmd_vel_msg.linear.x = 0.4;
    cmd_vel_msg.angular.z = 0.0;
  }
  // Call the publish_velocity function
  publish_velocity(std::make_shared<geometry_msgs::msg::Twist>(cmd_vel_msg));
}
