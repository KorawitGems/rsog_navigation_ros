cmake_minimum_required(VERSION 3.0.2)
project(pocv_local_planner_ros)

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
  aogm_planner_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pocv_local_planner_ros
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros aogm_planner_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library( pocv_local_planner_ros
    src/pocv_local_planner_ros.cpp 
)
target_link_libraries( pocv_local_planner_ros
    aogm_common_map
    aogm_planner_ros
    ${catkin_LIBRARIES}
)

add_executable(pocv_local_planner_ros_node
  src/pocv_local_planner_ros_node.cpp
)
target_link_libraries(pocv_local_planner_ros_node
  pocv_local_planner_ros
  ${catkin_LIBRARIES}
)