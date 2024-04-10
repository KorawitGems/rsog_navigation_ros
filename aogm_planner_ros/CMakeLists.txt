cmake_minimum_required(VERSION 3.0.2)
project(aogm_planner_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aogm_common_map aogm_planner_ros
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( aogm_common_map
    src/map/aogm_common_map.cpp
)
target_link_libraries( aogm_common_map
    ${catkin_LIBRARIES}
)  

add_library( aogm_planner_ros
    src/planner/aogm_planner_ros.cpp
)
target_link_libraries( aogm_planner_ros
    aogm_common_map
    ${catkin_LIBRARIES}
)
