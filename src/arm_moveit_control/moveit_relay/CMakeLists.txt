cmake_minimum_required(VERSION 3.8)
project(moveit_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到所有依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_msg REQUIRED)

# 编译节点
add_executable(moveit_relay_node src/arm_motor_control.cpp)
ament_target_dependencies(moveit_relay_node
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs
  trajectory_msgs
  robot_msg
)

# 安装
install(TARGETS
  moveit_relay_node
  DESTINATION lib/${PROJECT_NAME}
  
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()