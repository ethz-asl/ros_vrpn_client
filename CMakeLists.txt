cmake_minimum_required(VERSION 2.8.0)

project(ros_vrpn_client)

find_package(catkin_simple REQUIRED)
catkin_simple()

find_package(Bullet REQUIRED)

include_directories(${BULLET_INCLUDE_DIR})

cs_add_executable(ros_vrpn_client 
	src/ros_vrpn_client.cpp
)

target_link_libraries(ros_vrpn_client 
    ${BULLET_MATH_LIBRARY} pthread
)

if(NOT DEFINED vrpn_catkin_DIR )
target_link_libraries(ros_vrpn_client 
    vrpn
)
endif()

cs_install()
cs_export()

