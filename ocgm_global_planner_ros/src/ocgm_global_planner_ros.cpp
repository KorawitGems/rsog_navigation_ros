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

#include <ocgm_global_planner_ros/ocgm_global_planner_ros.hpp>

OCGMGlobalPlannerROS::OCGMGlobalPlannerROS() : pnh_("~"), tf_listener_(tf_buffer_) {
    map_sub_ = nh_.subscribe("/map", 1, &OCGMGlobalPlannerROS::map_callback, this);
    goal_sub_ = nh_.subscribe("/goal", 1, &OCGMGlobalPlannerROS::goal_callback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/global_planner/path", 1);
    current_node_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/global_planner/current_node", 1);
    occupancy_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_planner/occupancy_cloud", 1);
    robot_radius_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_planner/robot_radius_cloud", 1);
    obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_planner/obstacle_cloud", 1);
    pnh_.param<std::string>("map_frame", param_map_frame_, "map");
    pnh_.param<std::string>("odom_frame", param_odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", param_base_frame_, "base_footprint");
    pnh_.param<double>("obstacle_map/radius", inflated_obstacle_radius_, 0.7);
    pnh_.param<double>("obstacle_map/weight", inflated_obstacle_weight_, 100.0);
    pnh_.param<double>("obstacle_map/robot_radius", inflated_robot_radius_, 0.3);
    pnh_.param<double>("obstacle_map/robot_weight", inflated_robot_weight_, 1000.0);
    pnh_.param<double>("goal_cost/weight", goal_cost_weight_, 5.0);
    pnh_.param<double>("move_cost/weight", move_cost_weight_, 10.0);

    ComponentMapState_ = false;
    timer_current_pose_ = nh_.createTimer(ros::Duration(0.02), &OCGMGlobalPlannerROS::getRobotPositionCallback, this);
    timer_pub_ = nh_.createTimer(ros::Duration(1.0), &OCGMGlobalPlannerROS::timerPublish, this);
    
    occupancy_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    robot_radius_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    obstacle_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

bool OCGMGlobalPlannerROS::isComponentMapReady() { 
    return ComponentMapState_; 
}

void OCGMGlobalPlannerROS::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!isComponentMapReady()){

        common_map_.map_msg_ = *msg;
        occupancy_map_.resize(common_map_.map_msg_.info.width * common_map_.map_msg_.info.height);
        obstacle_map_.resize(common_map_.map_msg_.info.width * common_map_.map_msg_.info.height);
        int index_obstacle_radius = std::round(inflated_obstacle_radius_ / common_map_.map_msg_.info.resolution);
        int index_robot_radius = std::round(inflated_robot_radius_ / common_map_.map_msg_.info.resolution);

        double occupancy_real_position_x;
        double occupancy_real_position_y;
        double robot_radius_real_position_x;
        double robot_radius_real_position_y;
        double obstacle_real_position_x;
        double obstacle_real_position_y;

        for (int i = 0; i < common_map_.map_msg_.info.width; i++) {
            for (int j = 0; j < common_map_.map_msg_.info.height; j++) {
                occupancy_map_[j * common_map_.map_msg_.info.width + i] = common_map_.map_msg_.data[j * common_map_.map_msg_.info.width + i];
                obstacle_map_[j * common_map_.map_msg_.info.width + i] = 1.0;
            }
        }

        for (int i = 0; i < common_map_.map_msg_.info.width; i++) {
            for (int j = 0; j < common_map_.map_msg_.info.height; j++) {
                if (common_map_.map_msg_.data[j * common_map_.map_msg_.info.width + i] == 100) {

                    // common_map_.mapIndexToPosition(i, j, occupancy_real_position_x, occupancy_real_position_y);
                    // pcl::PointXYZI occupancy_point;
                    // occupancy_point.x = occupancy_real_position_x;
                    // occupancy_point.y = occupancy_real_position_y;
                    // occupancy_point.z = 0.0;
                    // occupancy_point.intensity = occupancy_map_[j * common_map_.map_msg_.info.width + i];
                    // occupancy_cloud_ptr_->points.emplace_back(occupancy_point);
                    
                    for (int dx = -index_obstacle_radius; dx <= index_obstacle_radius; dx++) {
                        for (int dy = -index_obstacle_radius; dy <= index_obstacle_radius; dy++) {
                            int x = i + dx;
                            int y = j + dy;
                            if (common_map_.isInMap(x,y)){
                                int index_distance = dx;
                                if (std::abs(dx) < std::abs(dy)) {
                                    index_distance = dy;
                                }
                                double distance = std::abs(index_distance) * common_map_.map_msg_.info.resolution;
                                if (std::abs(dx) <= index_robot_radius && std::abs(dy) <= index_robot_radius) {
                                    occupancy_map_[y * common_map_.map_msg_.info.width + x] = 100;
                                    common_map_.mapIndexToPosition(x, y, robot_radius_real_position_x, robot_radius_real_position_y);
                                    pcl::PointXYZI robot_radius_point;
                                    robot_radius_point.x = robot_radius_real_position_x;
                                    robot_radius_point.y = robot_radius_real_position_y;
                                    robot_radius_point.z = 0.0;
                                    robot_radius_point.intensity = 100; // debug
                                    robot_radius_cloud_ptr_->points.emplace_back(robot_radius_point);
                                }
                                if (distance < inflated_obstacle_radius_) {
                                    double decayed_cost = inflated_obstacle_weight_*(1.0 - std::pow( distance / inflated_obstacle_radius_, 3));
                                    obstacle_map_[y * common_map_.map_msg_.info.width + x] = std::max(decayed_cost + 2.0, obstacle_map_[y * common_map_.map_msg_.info.width + x]);
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < common_map_.map_msg_.info.width; i++) {
            for (int j = 0; j < common_map_.map_msg_.info.height; j++) {
                if (occupancy_map_[j * common_map_.map_msg_.info.width + i] == 100) {
                    common_map_.mapIndexToPosition(i, j, robot_radius_real_position_x, robot_radius_real_position_y);
                    pcl::PointXYZI robot_radius_point;
                    robot_radius_point.x = robot_radius_real_position_x;
                    robot_radius_point.y = robot_radius_real_position_y;
                    robot_radius_point.z = 0.0;
                    robot_radius_point.intensity = 100; // debug
                    robot_radius_cloud_ptr_->points.emplace_back(robot_radius_point);
                }
                pcl::PointXYZI obstacle_point;
                obstacle_point.intensity = (obstacle_map_[j * common_map_.map_msg_.info.width + i] - 2.0)/inflated_obstacle_weight_*255; // debug
                if (obstacle_point.intensity > 0){
                    common_map_.mapIndexToPosition(i, j, obstacle_real_position_x, obstacle_real_position_y);
                    obstacle_point.x = obstacle_real_position_x;
                    obstacle_point.y = obstacle_real_position_y;
                    obstacle_point.z = 0.0;
                    obstacle_cloud_ptr_->points.emplace_back(obstacle_point);
                }
            }
        }

        // pcl::toROSMsg(*occupancy_cloud_ptr_, occupancy_cloud_msg_);
        // occupancy_cloud_msg_.header.frame_id = param_map_frame_;

        pcl::toROSMsg(*robot_radius_cloud_ptr_, robot_radius_cloud_msg_);
        robot_radius_cloud_msg_.header.frame_id = param_map_frame_;

        pcl::toROSMsg(*obstacle_cloud_ptr_, obstacle_cloud_msg_);
        obstacle_cloud_msg_.header.frame_id = param_map_frame_;

        ComponentMapState_ = true;
    }
}

void OCGMGlobalPlannerROS::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_ = *msg;
    common_map_.positionToMapIndex(msg->pose.position.x, msg->pose.position.y, planner_.goal_index_.x, planner_.goal_index_.y);
    nav_msgs::Path result_path;
    bool found_path = false;
    ros::Time last_time_global_plan = ros::Time::now();
    while (ros::Time::now() - last_time_global_plan < ros::Duration(1.0)) {
        ros::Time start_time = ros::Time::now();
        found_path = planner_.plan_path(common_map_, occupancy_map_, obstacle_map_, 
                                                    goal_cost_weight_, move_cost_weight_, result_path);
        if (found_path) {
            ROS_WARN("Global planner time execution : %f seconds", (ros::Time::now() - start_time).toSec());
            publish_path(result_path);
            break;
        }
    }
    if (!found_path){
        ROS_WARN("No valid global path found!");
    }
    //publish_current_node(planner_.current_node_pose_array_); //debug
}

void OCGMGlobalPlannerROS::getRobotPositionCallback(const ros::TimerEvent& event) {
    geometry_msgs::TransformStamped base_in_map_tf_msg;
    try {
        ros::Rate r(100);
        ros::Time last_time = ros::Time::now();
        while(!tf_buffer_.canTransform(param_map_frame_, param_base_frame_, ros::Time(0))) {
            if ((ros::Time::now() - last_time) > ros::Duration(0.1)){
                ROS_WARN("Global planner time up 0.1 sec. Transform from %s to %s not available.", param_base_frame_.c_str(), param_map_frame_.c_str());
                last_time = ros::Time::now();
            }
            r.sleep();
        }
        base_in_map_tf_msg = tf_buffer_.lookupTransform(param_map_frame_, param_base_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    common_map_.positionToMapIndex(base_in_map_tf_msg.transform.translation.x, base_in_map_tf_msg.transform.translation.y,
                            planner_.start_index_.x, planner_.start_index_.y);
}

void OCGMGlobalPlannerROS::timerPublish(const ros::TimerEvent& event)
{
    if (isComponentMapReady()) {
        // occupancy_cloud_msg_.header.stamp = ros::Time::now();
        // occupancy_cloud_pub_.publish(occupancy_cloud_msg_);

        robot_radius_cloud_msg_.header.stamp = ros::Time::now();
        robot_radius_cloud_pub_.publish(robot_radius_cloud_msg_);

        obstacle_cloud_msg_.header.stamp = ros::Time::now();
        obstacle_cloud_pub_.publish(obstacle_cloud_msg_);
    }
}

void OCGMGlobalPlannerROS::publish_path(nav_msgs::Path& path_msg) {
    path_msg.header.frame_id = param_map_frame_;
    path_msg.header.stamp = ros::Time::now();
    // for (int i = 0; i < path_msg.poses.size(); i++) {
    //     path_msg.poses[i].pose.orientation = goal_.pose.orientation;
    // }
    path_pub_.publish(path_msg);
}

void OCGMGlobalPlannerROS::publish_current_node(std::vector<geometry_msgs::PoseStamped>& current_node_pose_array) {
    for (auto& current_pose : current_node_pose_array) {
        current_pose.header.stamp = ros::Time::now();
        current_node_pub_.publish(current_pose);
        ros::Duration(0.01).sleep();
    }
}

void OCGMGlobalPlannerROS::run() {
    ros::Duration(1.0).sleep();
    ros::Rate rate(10);
    while (!isComponentMapReady()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Global Planner Ready to plan");
    ros::spin();
}



