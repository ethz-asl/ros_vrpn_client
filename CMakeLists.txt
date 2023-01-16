cmake_minimum_required(VERSION 3.5)
project(ros_vrpn)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vrpn_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ViconEstimator.msg
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Compile library
add_library(vicon_estimator
  src/library/vicon_estimator.cpp
  src/wrapper/vicon_odometry_estimator.cpp
)

rclcpp_components_register_node(vicon_estimator PLUGIN "vicon_estimator::ViconOdometryEstimator" EXECUTABLE vicon_odometry_estimator)


target_include_directories(vicon_estimator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(vicon_estimator
  rclcpp
  rclcpp_components
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

target_link_libraries(vicon_estimator "${cpp_typesupport_target}")


ament_export_dependencies(
  rosidl_default_runtime
  eigen3_cmake_module
  Eigen3
)

# Compile ros_vrpn_client
add_executable(ros_vrpn_client
  src/ros_vrpn_client.cpp
)

target_include_directories(ros_vrpn_client
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(ros_vrpn_client vicon_estimator)

ament_target_dependencies(ros_vrpn_client
  rclcpp
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  vrpn_vendor
  nav_msgs
  geometry_msgs
)

# Compile vicon_estimation_node
add_executable(vicon_estimation_node
  src/vicon_estimation_node.cpp
)

target_include_directories(vicon_estimation_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(vicon_estimation_node vicon_estimator)

ament_target_dependencies(vicon_estimation_node
  rclcpp
  Eigen3
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  vrpn_vendor
  nav_msgs
  geometry_msgs
)

# Install targets
install(
  TARGETS vicon_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ros_vrpn_client vicon_estimation_node
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Unit tests
  add_subdirectory(src/test)

  # Test linting
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()