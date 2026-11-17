cmake_minimum_required(VERSION 3.5)
project(tai_robot_one)

# Export the exact compiler flags and versioned ROS include paths for
# VS Code IntelliSense and other C++ tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# The ros2_control Jazzy hardware API and this serial backend use C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp QUIET)
find_package(geometry_msgs REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller QUIET)
find_package(nav2_core QUIET)
find_package(nav2_smac_planner QUIET)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(nav2_regulated_pure_pursuit_controller_FOUND AND nav2_core_FOUND)
  add_library(tai_safe_rotation_rpp SHARED src/safe_rotation_rpp.cpp)
  target_include_directories(tai_safe_rotation_rpp PRIVATE include)
  ament_target_dependencies(tai_safe_rotation_rpp
    nav2_regulated_pure_pursuit_controller nav2_core tf2_geometry_msgs pluginlib)
  pluginlib_export_plugin_description_file(nav2_core tai_navigation_plugins.xml)
endif()

if(nav2_smac_planner_FOUND AND nav2_core_FOUND)
  add_library(tai_exact_goal_smac_planner SHARED src/exact_goal_smac_planner.cpp)
  target_include_directories(tai_exact_goal_smac_planner PRIVATE include)
  ament_target_dependencies(tai_exact_goal_smac_planner
    nav2_smac_planner nav2_core nav2_costmap_2d nav_msgs geometry_msgs pluginlib)
  if(NOT TARGET tai_safe_rotation_rpp)
    pluginlib_export_plugin_description_file(nav2_core tai_navigation_plugins.xml)
  endif()
endif()

# Fast DDS is the validated middleware for the current WSL mirrored-network
# setup and the Raspberry Pi deployment. Install a workspace environment hook
# so every terminal that sources install/setup.bash selects the same RMW.
# Preserve an explicit user override for deployments that require another RMW.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env/tai_robot_one_middleware.sh.in"
)

add_library(tai_robot_one_hardware SHARED
  src/tai_robot_serial_system.cpp
)
target_compile_features(tai_robot_one_hardware PUBLIC cxx_std_17)
target_include_directories(tai_robot_one_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tai_robot_one_hardware
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
ament_target_dependencies(tai_robot_one_hardware std_msgs)

pluginlib_export_plugin_description_file(
  hardware_interface tai_robot_one_hardware.xml)

# BT helper: return a recovery angle opposite to the last commanded turn.
if(behaviortree_cpp_FOUND)
  add_library(tai_path_footprint_clear_bt_node SHARED src/path_footprint_clear_bt_node.cpp)
  target_include_directories(tai_path_footprint_clear_bt_node PRIVATE include)
  target_compile_definitions(tai_path_footprint_clear_bt_node PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(tai_path_footprint_clear_bt_node
    behaviortree_cpp rclcpp geometry_msgs nav_msgs nav2_msgs nav2_costmap_2d
    tf2_geometry_msgs tf2_ros)
  add_library(tai_compute_opposite_spin_bt_node SHARED
    src/compute_opposite_spin_bt_node.cpp
  )
  target_compile_features(tai_compute_opposite_spin_bt_node PUBLIC cxx_std_17)
  target_compile_definitions(tai_compute_opposite_spin_bt_node
    PRIVATE BT_PLUGIN_EXPORT
  )
  ament_target_dependencies(tai_compute_opposite_spin_bt_node
    behaviortree_cpp
    geometry_msgs
    rclcpp
  )
endif()


install(
  DIRECTORY behavior_trees config description docs launch udev web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  DIRECTORY include/
  DESTINATION include
)

set(tai_install_targets tai_robot_one_hardware)
foreach(optional_target
    tai_compute_opposite_spin_bt_node tai_safe_rotation_rpp
    tai_path_footprint_clear_bt_node tai_exact_goal_smac_planner)
  if(TARGET ${optional_target})
    list(APPEND tai_install_targets ${optional_target})
  endif()
endforeach()

install(
  TARGETS ${tai_install_targets}
  EXPORT export_tai_robot_one_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/cmd_vel_stamper
    scripts/amcl_pose_persistence
    scripts/wait_for_localization_tf
    scripts/publish_rgb_calibrated_cloud.py
    scripts/nav_path_display_filter
    scripts/cm029_teleop
    scripts/scan_self_filter
    scripts/keyboard_teleop
    scripts/lift_position_controller
    scripts/phone_teleop_server
    scripts/imu_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Share the exact per-capture depth geometry with pallet recognition instead
# of maintaining a second, static-skew obstacle-cloud approximation.
install(
  FILES
    ../camera/detect_astra/scripts/pallet_plane_heading.py
    ../camera/detect_astra/scripts/pallet_target_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

# The GitHub esp32_imu_bridge package is retained under src/ for provenance,
# but this repository is itself one ament package, so colcon does not descend
# into that nested package. Install its node as part of tai_robot_one so the
# bridge is actually runnable from this workspace.
install(
  PROGRAMS
    src/esp32_imu_bridge/esp32_imu_bridge/esp32_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME esp32_imu_bridge
)

ament_export_targets(export_tai_robot_one_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

ament_package()
