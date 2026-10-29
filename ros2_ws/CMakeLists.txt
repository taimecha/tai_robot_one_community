cmake_minimum_required(VERSION 3.5)
project(tai_robot_one)

# Export the exact compiler flags and versioned ROS / Gazebo include paths for
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
find_package(gz_sim_vendor QUIET)
find_package(gz-sim8 QUIET)
find_package(gz_transport_vendor QUIET)
find_package(gz-transport13 QUIET)
find_package(gz_msgs_vendor QUIET)
find_package(gz-msgs10 QUIET)
find_package(behaviortree_cpp QUIET)
find_package(geometry_msgs REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller QUIET)
find_package(nav2_core QUIET)
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

# Rendering-independent 2D lidar for Gazebo Harmonic.  It uses DART's
# RaycastData component, allowing the RGB-D camera to stay on Ogre1 under WSL
# without triggering Ogre1's broken GPU-ray implementation.
if(TARGET gz-sim8::gz-sim8 AND TARGET gz-transport13::gz-transport13 AND
   TARGET gz-msgs10::gz-msgs10)
  add_library(tai_physics_lidar SHARED
    src/tai_physics_lidar.cpp
  )
  target_compile_features(tai_physics_lidar PUBLIC cxx_std_17)
  target_link_libraries(tai_physics_lidar PUBLIC
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
  )
endif()

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

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_stop_envelope
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stop_envelope.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rotation_sweep test/test_rotation_sweep.cpp)
  target_include_directories(test_rotation_sweep PRIVATE include)
  if(TARGET tai_safe_rotation_rpp)
    ament_add_gtest(test_rear_goal_controller test/test_rear_goal_controller.cpp)
    target_include_directories(test_rear_goal_controller PRIVATE include)
    ament_target_dependencies(test_rear_goal_controller
      nav2_regulated_pure_pursuit_controller nav2_core tf2_geometry_msgs pluginlib)
  endif()
  if(TARGET tai_path_footprint_clear_bt_node)
    find_package(nlohmann_json REQUIRED)
    ament_add_gtest(test_path_footprint_bt test/test_path_footprint_bt.cpp)
    ament_target_dependencies(test_path_footprint_bt
      behaviortree_cpp rclcpp geometry_msgs nav_msgs nav2_msgs tf2_ros tf2_geometry_msgs)
    target_link_libraries(test_path_footprint_bt nlohmann_json::nlohmann_json)
    target_compile_definitions(test_path_footprint_bt PRIVATE
      FOOTPRINT_BT_LIBRARY="$<TARGET_FILE:tai_path_footprint_clear_bt_node>"
      REAL_ESCAPE_TREE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/navigate_real_clearance_escape.xml")
    add_dependencies(test_path_footprint_bt tai_path_footprint_clear_bt_node)
  endif()
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY behavior_trees config description docs launch udev web worlds
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
    tai_physics_lidar tai_compute_opposite_spin_bt_node tai_safe_rotation_rpp
    tai_path_footprint_clear_bt_node)
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
    scripts/astra_sim_range_filter
    scripts/publish_rgb_calibrated_cloud.py
    scripts/nav_path_display_filter
    scripts/cm029_teleop
    scripts/scan_self_filter
    scripts/keyboard_teleop
    scripts/lift_position_controller
    scripts/phone_teleop_server
    scripts/imu_visualizer
    scripts/turn_relative_yaw
    scripts/test_robot_motion
    scripts/test_nav_round_trip
    scripts/monitor_sim_clearance
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
