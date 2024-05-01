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

#ifndef POCV_LOCAL_PLANNER_ROS_HPP
#define POCV_LOCAL_PLANNER_ROS_HPP

#include <aogm_planner_ros/map/aogm_common_map.hpp>
#include <aogm_planner_ros/planner/aogm_planner_ros.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h> 
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <vector>

struct Cost {
    double goal;
    double obstacle;
    double path;
    double total;
};

enum State {
    WAIT_FOR_GOAL,
    ACHIEVE_GOAL,
    Heading_To_Goal,
    CHECK_GOAL_POSITION,
    NEAR_GOAL,
    ACHIEVE_GLOBAL_POINT,
    TIME_UP_GLOBAL_POINT,
    Heading_To_GLOBAL_POINT,
    Heading_To_LOCAL_POINT,
    PLAN_PATH,
    PATH_INTERSECT_OBSTACLE,
    SIMULATE_VELOCITY
};

class POCVLocalPlannerROS {
public:
    POCVLocalPlannerROS();
    void run();

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    bool createPathMap();
    bool doPathIntersectObstacle();
    bool findBestVelocity(geometry_msgs::Twist& output_predict_cmd_vel);
    void calculateCost(const geometry_msgs::Twist& predict_cmd_vel, const double& punitive_cost, Cost& output_cost);
    void findAngularCmdVel(geometry_msgs::Twist& predict_cmd_vel);
    void publishCloud();
    void timerPublish(const ros::TimerEvent& event);
    void updatePlanner();
    void handleLookupTransform(const std::string& target_frame, const std::string& source_frame, geometry_msgs::TransformStamped& output_transform_stamped);
    void publish_path(nav_msgs::Path& path_msg);

    ros::NodeHandle nh_, pnh_;
    ros::Subscriber global_path_sub_, goal_sub_, map_sub_, odom_subscriber_, scan_sub_;
    ros::Publisher cmd_vel_pub_, local_path_pub_, local_point_pub_, global_point_pub_, sim_poses_pub_, sim_vels_pub_, robot_radius_cloud_pub_, obstacle_cloud_pub_, path_cloud_pub_;
    ros::Timer timer_pub_;
    nav_msgs::Odometry odom_msg_;
    nav_msgs::Path global_path_, local_path_, heading_local_path_;
    sensor_msgs::LaserScan laser_msg_;
    geometry_msgs::TransformStamped base_in_map_tf_msg_, laser_in_map_tf_msg_, last_base_in_map_tf_msg_;
    geometry_msgs::PoseStamped goal_, last_global_point_;
    geometry_msgs::Twist predict_cmd_vel_, last_cmd_vel_;
    geometry_msgs::PoseArray predict_base_msg_;
    std::vector<double> path_map_, laser_map_;
    std::vector<int> occupancy_map_, local_map_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    tf2::Quaternion q_local_point_, q_global_point_;
    ros::Time last_time_, start_time_, last_time_local_goal_, last_time_intersect_, last_time_achieve_goal_, last_time_global_point_in_obs_, last_time_skip_global_point_;
    bool receive_laser_msg_, receive_map_msg_, receive_odom_msg_, receive_path_map_, is_initial_, use_path_cost_, use_heading_to_local_point_, wait_heading_to_local_point_;
    bool use_temp_variable_, debug_time_execution_, debug_simulated_variable_;
    std::string param_map_frame_, param_odom_frame_, param_base_frame_;
    int index_path_radius_, index_laser_radius_, index_robot_radius_, index_local_planner_;
    double time_step_, sim_time_step_, linear_resolution_, angular_resolution_, base_yaw_, global_point_yaw_, last_base_yaw_;
    double min_linear_velocity_, max_linear_velocity_, min_angular_velocity_, max_angular_velocity_; 
    double max_abs_linear_acceleration_, max_abs_angular_acceleration_;
    double vel_obstacle_weight_, vel_goal_weight_, vel_path_weight_;
    double inflated_path_radius_, inflated_path_weight_, inflated_laser_radius_, inflated_laser_weight_, inflated_robot_radius_, inflated_robot_weight_, inflated_local_planner_;
    double planner_goal_weight_, planner_move_weight_;
    double delta_yaw_from_current_base_, global_point_tolerance_, goal_position_tolerance_, goal_orientation_tolerance_;
    double temp_vel_goal_weight_, temp_global_point_tolerance_;
    double cmd_ang_vz_, min_rised_cost_;
    AOGMCommonMap common_map_;
    AOGMPlannerROS planner_;
    Cost last_best_cost_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sim_base_cloud_ptr_, sim_vel_cloud_ptr_, robot_radius_cloud_ptr_, obstacle_cloud_ptr_, path_cloud_ptr_;
    State state_;
};

#endif