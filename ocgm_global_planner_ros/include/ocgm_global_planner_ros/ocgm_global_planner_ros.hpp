/* BSD 3-Clause License

Copyright (c) 2024, Korawit Kitikangsadan

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef OCGM_GLOBAL_PLANNER_ROS_HPP
#define OCGM_GLOBAL_PLANNER_ROS_HPP

#include <aogm_planner_ros/map/aogm_common_map.hpp>
#include <aogm_planner_ros/planner/aogm_planner_ros.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OCGMGlobalPlannerROS {

public:
    OCGMGlobalPlannerROS();
    void run();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher current_node_pub_;
    ros::Publisher occupancy_cloud_pub_;
    ros::Publisher robot_radius_cloud_pub_;
    ros::Publisher obstacle_cloud_pub_;
    ros::Timer timer_current_pose_, timer_pub_;
    geometry_msgs::PoseStamped goal_;
    AOGMCommonMap common_map_;
    AOGMPlannerROS planner_;
    std::vector<int> occupancy_map_;
    std::vector<double> obstacle_map_;
    std::string param_map_frame_, param_odom_frame_, param_base_frame_;
    double inflated_obstacle_radius_, inflated_obstacle_weight_;
    double inflated_robot_radius_, inflated_robot_weight_;
    double goal_cost_weight_, move_cost_weight_, plan_time_up_;
    bool ComponentMapState_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    // std::chrono::high_resolution_clock::time_point start_time_;
    // std::chrono::high_resolution_clock::time_point end_time_;
    // std::chrono::duration<double, std::milli> timeTaken_;
    sensor_msgs::PointCloud2 occupancy_cloud_msg_, robot_radius_cloud_msg_, obstacle_cloud_msg_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud_ptr_, robot_radius_cloud_ptr_, obstacle_cloud_ptr_;

    bool isComponentMapReady();
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void getRobotPositionCallback(const ros::TimerEvent& event);
    void timerPublish(const ros::TimerEvent& event);
    void publish_path(nav_msgs::Path& path_msg);
    void publish_current_node(std::vector<geometry_msgs::PoseStamped>& current_node_pose_array);
};

#endif