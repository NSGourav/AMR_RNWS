cmake_minimum_required(VERSION 3.8)
project(rn_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Build the shared library
add_library(diffdrive_hardware SHARED
  src/diffdrive_hardware_interface.cpp
)

target_include_directories(diffdrive_hardware PRIVATE include)
ament_target_dependencies(diffdrive_hardware
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(diffdrive_hardware
  ${rclcpp_LIBRARIES}
  ${pluginlib_LIBRARIES}
  ${hardware_interface_LIBRARIES}
)


# Export pluginlib file
pluginlib_export_plugin_description_file(hardware_interface resource/diffdrive_hardware_plugin.xml)

# Install targets and plugin description
install(TARGETS diffdrive_hardware
  DESTINATION lib
)

install(FILES
  resource/diffdrive_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/odom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/goal_to_goal_nav.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
