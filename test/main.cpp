#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE WITH TESTING BYE BYE" << std::endl;
  return result;
}