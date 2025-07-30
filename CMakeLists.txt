cmake_minimum_required(VERSION 3.8)
project(autoware_zed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(zed_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(autoware_zed_component SHARED
  src/autoware_zed_node.cpp
)

target_compile_features(autoware_zed_component PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(autoware_zed_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  zed_msgs
  autoware_perception_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Register component
rclcpp_components_register_nodes(autoware_zed_component
  "autoware_zed::AutowareZedNode"
)

# Create executable
add_executable(autoware_zed_node
  src/autoware_zed_main.cpp
)

target_link_libraries(autoware_zed_node
  autoware_zed_component
)

ament_target_dependencies(autoware_zed_node
  rclcpp
)

# Install
install(TARGETS
  autoware_zed_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  autoware_zed_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(autoware_zed_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  zed_msgs
  autoware_perception_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()