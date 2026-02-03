cmake_minimum_required(VERSION 3.5)
project(grab_it_gz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES 'Clang')
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
#find_package(gazebo_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)


install(DIRECTORY config launch model
  DESTINATION share/${PROJECT_NAME}/
)

# ament package macros
ament_package()
