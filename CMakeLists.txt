cmake_minimum_required(VERSION 3.8)
project(hyq2max_ros2_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(hyq2max_interfaces REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hyq2max_ros2_impedance_controller.cpp
)

# add_executable(hyq2max_impedance_controller
#  src/hyq2max_ros2_impedance_controller.cpp
# )

target_include_directories( ${PROJECT_NAME} PRIVATE include
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  nav_msgs
  hyq2max_interfaces
  control_toolbox
)

ament_target_dependencies( ${PROJECT_NAME}
  hyq2max_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

pluginlib_export_plugin_description_file(controller_interface hyq2max_ros2_impedance_controller.xml)

install( TARGETS ${PROJECT_NAME} DESTINATION 
  lib
)

install( DIRECTORY include DESTINATION 
  include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  hyq2max_ros2_impedance_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
