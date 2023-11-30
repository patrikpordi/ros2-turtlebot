cmake_minimum_required(VERSION 3.8)
project(ros2-turtlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# add_library(walking_lib SHARED lib/walking.cpp)
add_library(walking_lib lib/walking.cpp)
target_include_directories(walking_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(walking_lib  rclcpp std_msgs geometry_msgs sensor_msgs)
ament_export_targets(walking_lib HAS_LIBRARY_TARGET)

# install the include directory and the library itself
install(
  DIRECTORY include
  DESTINATION include
)
install(
  TARGETS walking_lib
  EXPORT walking_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# executable
add_executable(walking src/main.cpp)
target_include_directories(walking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(walking walking_lib)
ament_target_dependencies(walking rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS walking
  DESTINATION lib/${PROJECT_NAME})
# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
