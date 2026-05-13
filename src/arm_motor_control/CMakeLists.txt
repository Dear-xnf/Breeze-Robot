cmake_minimum_required(VERSION 3.8)
project(arm_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find dependencies ros依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(robot_msg REQUIRED)

#c++头文件

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# 库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


add_executable(arm_motor_control_node
    src/main.cpp
    src/arm_motor_control.cpp
    src/damiao_motor.cpp
    src/can_bus.cpp
    )



# ==========依赖连接============

ament_target_dependencies(arm_motor_control_node
  rclcpp
  sensor_msgs
  std_srvs   
  robot_msg
)

target_link_libraries(arm_motor_control_node
    ${EIGEN3_LIBRARIES}
    controlcan
)



install(TARGETS
  arm_motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装动态库
install(FILES
  lib/libcontrolcan.so
  DESTINATION lib/${PROJECT_NAME}
)


#加载库变量环境
set_target_properties(arm_motor_control_node PROPERTIES
  BUILD_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib
  INSTALL_RPATH "$ORIGIN"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
