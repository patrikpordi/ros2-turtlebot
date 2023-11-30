/**
 * @file walking.hpp
 * @author Patrik Dominik Pördi (ppordi@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Turtle_walking : public rclcpp::Node {
    private:
        double safety_distance;
        int rotate;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        /**
         * @brief Function to publish the velocity values
         * 
         * @param msg 
         */
        void publish_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);

        /**
         * @brief Function to act based on the laser scan data
         *
         * @param msg
         */
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    public:
        /**
         * @brief Construct a new Turtle walking object
         *
         */
        Turtle_walking()
            : Node("Walking_node"), safety_distance(0.4), rotate(-1) {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"),
                            "Setting up Walking Node...");
            // Set the collision threshold
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"),
                            "Safety distance: " << safety_distance);
            // Create a publisher for the cmd_vel topic
            publisher_ =
                this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 8);
            // Create a subscription to the laser scan topic
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", 10, std::bind(&Turtle_walking::scan_callback, this, _1));
            RCLCPP_INFO_STREAM(rclcpp::get_logger("Turtle_walking"),
                            "Walking Node Initialized!");
        }

    
        
  };