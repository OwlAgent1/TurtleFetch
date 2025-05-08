cmake_minimum_required(VERSION 3.8)
project(se3_sensor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)
add_library(se3_sensor_driver SHARED
  src/hardware_interface.cpp
)

ament_target_dependencies(se3_sensor_driver
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
)

add_executable(sensor_tf_sender src/sensor_tf_sender.cpp)
ament_target_dependencies(sensor_tf_sender
  tf2_ros
)
install(TARGETS sensor_tf_sender
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS se3_sensor_driver DESTINATION lib
)

install(DIRECTORY config launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

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
