
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

#ifndef APMC_LOCALIZATION_ROS_HPP
#define APMC_LOCALIZATION_ROS_HPP

#include <aogm_planner_ros/map/aogm_common_map.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class APMCLocalizationROS {
public:
    APMCLocalizationROS();
    ~APMCLocalizationROS();
    void run();

private:
    void initializeNode();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void generateSamples(double& linear_size, double& angular_size, 
                            double& linear_resolution, double& angular_resolution, geometry_msgs::PoseWithCovarianceStamped& initial_particle_pose);
    void calculateLikelihoodField(Particle& particle);
    void gaussianProbability(const double& z, const double& sigma, double& output_p_hit);
    void randomProbability(const double& z_rand, double& output_p_rand);
    void handleLocalization();
    void updateLocalization();
    void preparePublisher(const Particle& particle);
    void publishTfTimer(const ros::TimerEvent& event);
    void handleLookupTransform(const std::string& target_frame, const std::string& source_frame, geometry_msgs::TransformStamped& output_transform_stamped);
    void initializeProcess();
    
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber map_sub_, laser_sub_, odom_sub_, initial_pose_sub_;
    ros::Publisher apmc_pose_pub_, odom_in_map_pub_;
    ros::Timer loop_pub_timer_;
    sensor_msgs::LaserScan laser_msg_;
    nav_msgs::Odometry odom_msg_, current_odom_msg_;
    geometry_msgs::PoseWithCovarianceStamped initial_particle_pose_cov_, particle_in_map_msg_;
    geometry_msgs::Pose last_robot_pose_;
    geometry_msgs::TransformStamped odom_in_map_tf_msg_, laser_in_base_tf_msg_;
    std::vector<int> grid_map_;
    Particle max_weight_particle_;
    std::string param_map_frame_, param_odom_frame_, param_base_frame_;
    bool publish_tf_, receive_map_msg_, receive_laser_msg_, receive_odom_msg_, is_initial_, found_particle_;
    double param_hit_weight_, param_rand_weight_, param_sigma_hit_, param_linear_update_, param_angular_update_, total_weight_, inflated_occupied_radius_;
    double param_sample_linear_resolution_, param_sample_linear_size_, param_sample_angular_size_, param_sample_angular_resolution_;
    double param_resample_linear_resolution_, param_resample_linear_size_, param_resample_angular_size_, param_resample_angular_resolution_;
    double p_rand_, z_distance_tol_;
    int param_laser_step_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::vector<tf2::Transform> point_in_base_vector_;
    AOGMCommonMap common_map_;
};

#endif
