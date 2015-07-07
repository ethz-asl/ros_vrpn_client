cmake_minimum_required(VERSION 2.8.0)

project(ros_vrpn_client)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})

add_definitions(-std=c++11)

cs_add_executable(ros_vrpn_client 
	src/ros_vrpn_client.cpp
)

cs_install()
cs_export()

