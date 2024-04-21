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

#include <pocv_local_planner_ros/pocv_local_planner_ros.hpp>

POCVLocalPlannerROS::POCVLocalPlannerROS() : pnh_("~"), tf_listener_(tf_buffer_)
{
    // Initialize ROS parameters
    pnh_.param<double>("min_linear_velocity", min_linear_velocity_, -0.2);
    pnh_.param<double>("max_linear_velocity", max_linear_velocity_, 0.2);
    pnh_.param<double>("min_angular_velocity", min_angular_velocity_, -0.5);
    pnh_.param<double>("max_angular_velocity", max_angular_velocity_, 0.5);
    pnh_.param<double>("max_abs_linear_acceleration", max_abs_linear_acceleration_, 2.0);
    pnh_.param<double>("max_abs_angular_acceleration", max_abs_angular_acceleration_, 2.0);
    pnh_.param<double>("linear_vel_resolution", linear_resolution_, 0.01);
    pnh_.param<double>("sim_vel/sim_time_step", sim_time_step_, 0.1);
    pnh_.param<double>("sim_vel/obstacle_cost/weight", vel_obstacle_weight_, 5.0);
    pnh_.param<double>("sim_vel/goal_cost/weight", vel_goal_weight_, 1.0);
    pnh_.param<double>("sim_vel/path_cost/weight", vel_path_weight_, 5.0);
    pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
    pnh_.param<bool>("wait_heading_to_local_point", wait_heading_to_local_point_, true);
    pnh_.param<std::string>("map_frame", param_map_frame_, "map");
    pnh_.param<std::string>("odom_frame", param_odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", param_base_frame_, "base_footprint");
    pnh_.param<double>("laser_map/radius", inflated_laser_radius_, 1.0);
    pnh_.param<double>("laser_map/weight", inflated_laser_weight_, 100.0);
    pnh_.param<double>("laser_map/robot_radius", inflated_robot_radius_, 0.3);
    pnh_.param<double>("laser_map/robot_weight", inflated_robot_weight_, 1000.0);
    pnh_.param<double>("path_map/radius", inflated_path_radius_, 2.0);
    pnh_.param<double>("path_map/weight", inflated_path_weight_, 100.0);
    pnh_.param<double>("planner/goal_cost/weight", planner_goal_weight_, 5.0);
    pnh_.param<double>("planner/move_cost/weight", planner_move_weight_, 10.0);

    goal_sub_ = nh_.subscribe("/goal", 1, &POCVLocalPlannerROS::goalCallback, this);
    global_path_sub_ = nh_.subscribe("/global_planner/path", 1, &POCVLocalPlannerROS::globalPathCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &POCVLocalPlannerROS::laserCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &POCVLocalPlannerROS::mapCallback, this);
    odom_subscriber_ = nh_.subscribe("/odom", 1, &POCVLocalPlannerROS::odomCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_planner/path", 1);
    global_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_planner/global_point", 1);
    local_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_planner/local_point", 1);
    timer_pub_ = nh_.createTimer(ros::Duration(0.1), &POCVLocalPlannerROS::timerPublish, this);
    sim_poses_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_planner/simulated_pose", 1);
    sim_vels_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_planner/simulated_velocity", 1);

    sim_base_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    sim_vel_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    receive_laser_msg_ = false;
    receive_map_msg_ = false;
    receive_odom_msg_ = false;
    receive_path_map_ = false;
    is_initial_ = false;
    use_path_cost_ = true;
    use_heading_to_local_point_ = true;
    state_ = WAIT_FOR_GOAL;
    last_cmd_vel_.linear.x = 0.0;
    last_cmd_vel_.linear.y = 0.0;
    last_cmd_vel_.angular.z = 0.0;
    cmd_ang_vz_ = 0.0;
    last_best_cost_.obstacle = 0.0;
    global_point_tolerance_ = 0.3;
    temp_sim_time_step_ = -0.1;
    last_time_intersect_ = ros::Time::now();
    last_time_achieve_goal_ = ros::Time::now();
    last_time_global_point_in_obs_ = ros::Time::now();
    last_time_skip_global_point_ = ros::Time::now();
}

void POCVLocalPlannerROS::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { 
    if (receive_map_msg_){
        return;
    }
    common_map_.map_msg_ = *msg;
    index_laser_radius_ = static_cast<int>(std::floor(inflated_laser_radius_ / common_map_.map_msg_.info.resolution));
    index_robot_radius_ = static_cast<int>(std::floor(inflated_robot_radius_ / common_map_.map_msg_.info.resolution));
    index_path_radius_ = static_cast<int>(std::floor(inflated_path_radius_ / common_map_.map_msg_.info.resolution));
    inflated_local_planner_ = inflated_robot_radius_ - 3*common_map_.map_msg_.info.resolution;
    index_local_planner_ = static_cast<int>(std::floor(inflated_local_planner_ / common_map_.map_msg_.info.resolution));
    
    int size_map = common_map_.map_msg_.info.width * common_map_.map_msg_.info.height;
    path_map_.resize(size_map);
    laser_map_.resize(size_map);
    occupancy_map_.resize(size_map);
    local_map_.resize(size_map);

    double initial_laser_map_value = -0.001*inflated_laser_weight_; // set all value to negative to make low obstacle cost
    std::fill(laser_map_.begin(), laser_map_.end(), initial_laser_map_value);
    std::fill(occupancy_map_.begin(), occupancy_map_.end(), 0);
    std::fill(local_map_.begin(), local_map_.end(), 0);
    receive_map_msg_ = true;
}

void POCVLocalPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_msg_ = *msg; 
    receive_odom_msg_ = true;
}

void POCVLocalPlannerROS::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    laser_msg_ = *msg;
    handleLookupTransform(param_map_frame_, param_base_frame_, base_in_map_tf_msg_);
    handleLookupTransform(param_map_frame_, laser_msg_.header.frame_id, laser_in_map_tf_msg_);
    // set initial all value to negative to make low obstacle cost
    if (!global_path_.poses.empty()) { 
        if (std::abs(global_path_.poses[0].pose.position.x - last_global_point_.pose.position.x) > 0.1 &&
                std::abs(global_path_.poses[0].pose.position.y - last_global_point_.pose.position.y) > 0.1) {
            last_global_point_ = global_path_.poses[0];
            double initial_laser_map_value = -0.001*inflated_laser_weight_;
            std::fill(laser_map_.begin(), laser_map_.end(), initial_laser_map_value);
            std::fill(occupancy_map_.begin(), occupancy_map_.end(), 0);
            std::fill(local_map_.begin(), local_map_.end(), 0);
        }
    }
    // create laser map as obstacle cost map
    if (!laser_msg_.ranges.empty()){
        for (int i = 0; i < laser_msg_.ranges.size(); ++i) 
        {
            if (!std::isnan(laser_msg_.ranges[i]) && !std::isinf(laser_msg_.ranges[i]) && laser_msg_.ranges[i] < laser_msg_.range_max && laser_msg_.ranges[i] > laser_msg_.range_min) {
                double angle = laser_msg_.angle_min + i * laser_msg_.angle_increment;
                geometry_msgs::TransformStamped point_in_laser_tf_msg;
                point_in_laser_tf_msg.transform.translation.x = laser_msg_.ranges[i] * std::cos(angle);
                point_in_laser_tf_msg.transform.translation.y = laser_msg_.ranges[i] * std::sin(angle);
                point_in_laser_tf_msg.transform.translation.z = 0.0;
                tf2::convert(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.0), point_in_laser_tf_msg.transform.rotation);
                geometry_msgs::TransformStamped point_in_map_tf_msg;
                tf2::Transform point_in_laser_tf2, laser_in_map_tf2;
                tf2::convert(point_in_laser_tf_msg.transform, point_in_laser_tf2); 
                tf2::convert(laser_in_map_tf_msg_.transform, laser_in_map_tf2); 
                tf2::Transform point_in_map_tf2 = laser_in_map_tf2 * point_in_laser_tf2;
                point_in_map_tf_msg.transform = tf2::toMsg(point_in_map_tf2);

                int index_x, index_y;
                common_map_.positionToMapIndex(point_in_map_tf_msg.transform.translation.x, point_in_map_tf_msg.transform.translation.y, index_x, index_y);
                for (int dx = -index_laser_radius_; dx <= index_laser_radius_; ++dx) {
                    for (int dy = -index_laser_radius_; dy <= index_laser_radius_; ++dy) {
                        int x = index_x + dx;
                        int y = index_y + dy;
                        if (common_map_.isInMap(x, y)) {
                            int index_distance = dx;
                            if (std::abs(dx) < std::abs(dy)) {
                                index_distance = dy;
                            }
                            double distance = std::abs(index_distance) * common_map_.map_msg_.info.resolution;
                            if (std::abs(dx) <= index_robot_radius_ && std::abs(dy) <= index_robot_radius_ ){
                                occupancy_map_[x + y * common_map_.map_msg_.info.width] = 100;
                            }
                            if (std::abs(dx) <= index_local_planner_ && std::abs(dy) <= index_local_planner_ ){
                                local_map_[x + y * common_map_.map_msg_.info.width] = 100;
                            }
                            if (distance < inflated_laser_radius_) {
                                double decayed_cost = inflated_laser_weight_*std::pow(1.0 - (distance / inflated_laser_radius_),2); 
                                //ROS_WARN("decayed_cost inflated_laser = %f", decayed_cost);
                                laser_map_[x + y * common_map_.map_msg_.info.width] = std::max(decayed_cost, laser_map_[x + y * common_map_.map_msg_.info.width]);
                                //ROS_WARN("laser_map_ inflated_laser value = %f", laser_map_[x + y * common_map_.map_msg_.info.width]);
                            }
                        }
                    }
                }
            }
        }
    }
    
    receive_laser_msg_ = true;
}

void POCVLocalPlannerROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_ = *msg;
}

void POCVLocalPlannerROS::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    start_time_ = ros::Time::now();
    last_time_ = ros::Time::now() - ros::Duration(0.05); // assume first step time to calculate velocity and cost
    // collect point every given distance along the path
    geometry_msgs::PoseStamped start_current;
    start_current.pose.position.x = base_in_map_tf_msg_.transform.translation.x;
    start_current.pose.position.y = base_in_map_tf_msg_.transform.translation.y;
    global_path_.poses.clear();
    global_path_.poses.emplace_back(start_current);
    for (int i = 0; i < msg->poses.size(); i++){
        if (std::sqrt(std::pow(msg->poses[i].pose.position.x - global_path_.poses[global_path_.poses.size()-1].pose.position.x, 2) + 
                std::pow(msg->poses[i].pose.position.y - global_path_.poses[global_path_.poses.size()-1].pose.position.y, 2)) > 2.0){
            // double delta_yaw = std::atan2(msg->poses[i].pose.position.y - global_path_.poses[global_path_.poses.size()-1].pose.position.y,
            //                             msg->poses[i].pose.position.x - global_path_.poses[global_path_.poses.size()-1].pose.position.x);
            global_path_.poses.emplace_back(msg->poses[i]);
            // tf2::convert(tf2::Quaternion(tf2::Vector3(0, 0, 1), delta_yaw), global_path_.poses[global_path_.poses.size()-1].pose.orientation);
        }
    }
    for (int i = 0; i <= 3; i++) {
        global_path_.poses.emplace_back(goal_); // collect goal in last place and more to avoid time up to reach goal
    }
    last_global_point_ = global_path_.poses[0];
    global_path_.poses.erase(global_path_.poses.begin());
    state_ = Heading_To_GLOBAL_POINT;
    receive_path_map_ = false;
    last_time_local_goal_ = ros::Time::now();
}

bool POCVLocalPlannerROS::createPathMap() {
    int map_index_x, map_index_y; // check global point is in obstacle
    if (!global_path_.poses.empty()) {
        if (ros::Time::now() - last_time_global_point_in_obs_ > ros::Duration(1.0)) {
            common_map_.positionToMapIndex(global_path_.poses[0].pose.position.x, global_path_.poses[0].pose.position.y, map_index_x, map_index_y);
            if (local_map_[map_index_x + map_index_y * common_map_.map_msg_.info.width] == 100) {
                ROS_WARN("Global point is in obstacle, skip global point");
                global_path_.poses.erase(global_path_.poses.begin());
                last_time_global_point_in_obs_ = ros::Time::now();
                return false;
            }
        }
    }

    if (global_path_.poses.empty()) {
        return false;
    }
    // find local path planner
    local_path_.poses.clear(); // set start and goal in planner_
    common_map_.positionToMapIndex(global_path_.poses[0].pose.position.x, global_path_.poses[0].pose.position.y, planner_.goal_index_.x, planner_.goal_index_.y);
    common_map_.positionToMapIndex(base_in_map_tf_msg_.transform.translation.x, base_in_map_tf_msg_.transform.translation.y, planner_.start_index_.x, planner_.start_index_.y);
    ros::Time last_time_local_plan = ros::Time::now();
    bool found_path = false;
    while (ros::Time::now() - last_time_local_plan < ros::Duration(1.0)) {
        ros::Time start_plan_time = ros::Time::now();
        found_path = planner_.plan_path(common_map_, local_map_, laser_map_, 
                                                    planner_goal_weight_, planner_move_weight_, local_path_);
        if (found_path) {
            ROS_WARN("Found Local planner time execution : %f seconds", (ros::Time::now() - start_plan_time).toSec());
            publish_path(local_path_);
            break;
        }
    }
    if (!found_path) {
        if (ros::Time::now() - last_time_skip_global_point_ > ros::Duration(1.0) ) {
            ROS_WARN("No valid local path found, skip global point");
            global_path_.poses.erase(global_path_.poses.begin());
            last_time_skip_global_point_ = ros::Time::now();
        }
        return false;
    }
    // create path cost map
    double initial_path_map_value = 0.001*inflated_path_weight_;
    std::fill(path_map_.begin(), path_map_.end(), initial_path_map_value); //  set all value to positive to make high path cost
    
    int index_start;
    if (!local_path_.poses.empty()) {
        for (int idx = 0; idx < local_path_.poses.size(); idx++){
            index_start = idx;
            if (idx >= 1){
                break;
            }
        }
        heading_local_path_.poses.clear();
        heading_local_path_.poses.emplace_back(local_path_.poses[index_start]);
        for (int i = 0; i < local_path_.poses.size(); ++i) {
            if (std::sqrt(std::pow(local_path_.poses[i].pose.position.x - heading_local_path_.poses[heading_local_path_.poses.size()-1].pose.position.x, 2) + 
                    std::pow(local_path_.poses[i].pose.position.y - heading_local_path_.poses[heading_local_path_.poses.size()-1].pose.position.y, 2)) > 1.0){
                heading_local_path_.poses.emplace_back(local_path_.poses[i]); // collect point every 1.0 distance
            }
            int index_x, index_y;
            common_map_.positionToMapIndex(local_path_.poses[i].pose.position.x, local_path_.poses[i].pose.position.y, index_x, index_y);
            for (int dx = -index_path_radius_; dx <= index_path_radius_; ++dx) {
                for (int dy = -index_path_radius_; dy <= index_path_radius_; ++dy) {
                    int x = index_x + dx;
                    int y = index_y + dy;
                    if (common_map_.isInMap(x, y)) {
                        int index_distance = dx;
                        if (std::abs(dx) < std::abs(dy)) {
                            index_distance = dy;
                        }
                        double distance = std::abs(index_distance) * common_map_.map_msg_.info.resolution;
                        if (std::abs(dx) <= index_robot_radius_ && std::abs(dy) <= index_robot_radius_ ){
                            if (distance < inflated_robot_radius_) {
                                //double rised_cost = -(std::pow(i,2)/20)*inflated_path_weight_*(1.0-(distance / inflated_path_radius_));
                                double rised_cost = -(std::pow(i,1.5)/5)*inflated_path_weight_*std::pow(1.0-(distance / inflated_path_radius_),2);
                                //ROS_WARN("rised_cost robot_radius = %f", rised_cost);
                                path_map_[x + y * common_map_.map_msg_.info.width] = std::min(rised_cost, path_map_[x + y * common_map_.map_msg_.info.width]); // minus is inverse 1-(dis/total)
                                //ROS_WARN("path_map_ robot_radius value = %f", path_map_[x + y * common_map_.map_msg_.info.width]);
                            }
                        } else {
                            if (distance < inflated_path_radius_) {
                                double rised_cost = -inflated_path_weight_*std::pow(1.0-(distance / inflated_path_radius_),2);
                                //ROS_WARN("rised_cost = %f", rised_cost);
                                path_map_[x + y * common_map_.map_msg_.info.width] = std::min(rised_cost, path_map_[x + y * common_map_.map_msg_.info.width]); // minus is inverse 1-(dis/total)
                                //ROS_WARN("path_map_ value = %f", path_map_[x + y * common_map_.map_msg_.info.width]);
                            }
                        }
                    }
                }
            }
        }
        heading_local_path_.poses.erase(heading_local_path_.poses.begin());
    }
    return true;
}

bool POCVLocalPlannerROS::doPathIntersectObstacle() {
    if (!local_path_.poses.empty()) {
        for (int i = 0; i < local_path_.poses.size(); ++i) {
            int map_index_x, map_index_y;
            common_map_.positionToMapIndex(local_path_.poses[i].pose.position.x, local_path_.poses[i].pose.position.y, map_index_x, map_index_y);
            if (local_map_[map_index_x + map_index_y * common_map_.map_msg_.info.width] == 100) {
                return true;
            }
        }
    }
    return false;
}

bool POCVLocalPlannerROS::findBestVelocity(geometry_msgs::Twist& output_predict_cmd_vel) {
    bool found_vel = false;
    Cost best_cost;
    best_cost.total = 50000.0;
    double punitive_cost = 49000.0;
    sim_base_cloud_ptr_->points.clear();
    sim_vel_cloud_ptr_->points.clear();

    // aC_in_B ( pC_in_B = 0 )
    double min_lin_vx_from_lin_ax = (1.0*-max_abs_linear_acceleration_ + last_cmd_vel_.linear.y*last_cmd_vel_.angular.z)*time_step_ + last_cmd_vel_.linear.x;
    double max_lin_vx_from_lin_ax = (1.0*max_abs_linear_acceleration_ + last_cmd_vel_.linear.y*last_cmd_vel_.angular.z)*time_step_ + last_cmd_vel_.linear.x;
    double min_lin_vy_from_lin_ay = (1.0*-max_abs_linear_acceleration_ - last_cmd_vel_.linear.x*last_cmd_vel_.angular.z)*time_step_ + last_cmd_vel_.linear.y;
    double max_lin_vy_from_lin_ay = (1.0*max_abs_linear_acceleration_ - last_cmd_vel_.linear.x*last_cmd_vel_.angular.z)*time_step_ + last_cmd_vel_.linear.y;
    
    int index_min_lin_vx = static_cast<int>(std::floor(std::max(min_lin_vx_from_lin_ax, min_linear_velocity_) / linear_resolution_));
    int index_max_lin_vx = static_cast<int>(std::floor(std::min(max_lin_vx_from_lin_ax, max_linear_velocity_) / linear_resolution_));
    int index_min_lin_vy = static_cast<int>(std::floor(std::max(min_lin_vy_from_lin_ay, min_linear_velocity_) / linear_resolution_));
    int index_max_lin_vy = static_cast<int>(std::floor(std::min(max_lin_vy_from_lin_ay, max_linear_velocity_) / linear_resolution_));
    
    for (int index_lin_vx = index_min_lin_vx; index_lin_vx <= index_max_lin_vx; index_lin_vx++) {
        for (int index_lin_vy = index_min_lin_vy; index_lin_vy <= index_max_lin_vy; index_lin_vy++) {
            Cost cost;
            cost.goal = 0.0;
            cost.obstacle = 0.0;
            cost.path = 0.0;
            cost.total = 0.0;
            geometry_msgs::Twist predict_cmd_vel;
            predict_cmd_vel.linear.x = index_lin_vx * linear_resolution_;
            predict_cmd_vel.linear.y = index_lin_vy * linear_resolution_;
            predict_cmd_vel.angular.z = 0.0;
            
            calculateCost(predict_cmd_vel, punitive_cost, cost);
            if (cost.total < best_cost.total) {
                best_cost = cost;
                output_predict_cmd_vel = predict_cmd_vel;
                if (best_cost.total < punitive_cost) {
                    found_vel = true;
                }
                last_best_cost_ = best_cost;
            }
        }
    }
    //ROS_INFO("\033[1;31m obstacle_cost: %f,\033[1;32m goal_cost:  %f,\033[1;34m path_cost: %f\033[0m", best_cost.obstacle, best_cost.goal, best_cost.path);
    //ROS_INFO("Best total cost %f have vel %f, %f, %f", best_cost.total, output_predict_cmd_vel.linear.x, output_predict_cmd_vel.linear.y, output_predict_cmd_vel.angular.z);
    return found_vel;
}

void POCVLocalPlannerROS::calculateCost(const geometry_msgs::Twist& predict_cmd_vel, const double& punitive_cost, Cost& output_cost) {
    bool valid_vel = true;

    pcl::PointXYZI sim_vel_point;
    sim_vel_point.x = predict_cmd_vel.linear.x;
    sim_vel_point.y = predict_cmd_vel.linear.y;
    sim_vel_point.z = predict_cmd_vel.angular.z;
    sim_vel_point.intensity = 150; // debug
    sim_vel_cloud_ptr_->points.emplace_back(sim_vel_point);
    
    tf2::Quaternion q_rot, q_base_in_map, q_predict;
    tf2::convert(base_in_map_tf_msg_.transform.rotation, q_base_in_map);
    double delta_yaw = std::atan2(std::sin(predict_cmd_vel.angular.z * sim_time_step_), std::cos(predict_cmd_vel.angular.z * sim_time_step_));
    q_rot.setRPY(0.0, 0.0, delta_yaw);
    q_predict = q_rot*q_base_in_map;
    q_predict.normalize();
    double predict_yaw = tf2::getYaw(q_predict);
    double predict_x = base_in_map_tf_msg_.transform.translation.x + (predict_cmd_vel.linear.x * std::cos(predict_yaw) - predict_cmd_vel.linear.y * std::sin(predict_yaw)) * sim_time_step_;
    double predict_y = base_in_map_tf_msg_.transform.translation.y + (predict_cmd_vel.linear.y * std::cos(predict_yaw) + predict_cmd_vel.linear.x * std::sin(predict_yaw)) * sim_time_step_;

    pcl::PointXYZI sim_base_point;
    sim_base_point.x = predict_x;
    sim_base_point.y = predict_y;
    sim_base_point.z = 0.0;
    sim_base_point.intensity = 50; // debug
    sim_base_cloud_ptr_->points.emplace_back(sim_base_point);

    int index_x, index_y;
    common_map_.positionToMapIndex(predict_x, predict_y, index_x, index_y); 
    if (common_map_.isInMap(index_x, index_y)){
        if (local_map_[index_x + index_y * common_map_.map_msg_.info.width] == 100){
        //ROS_WARN("Local planner crash obstacle");
        output_cost.total = punitive_cost;
        valid_vel = false;
        } else {
            output_cost.obstacle = laser_map_[index_x + index_y * common_map_.map_msg_.info.width] * vel_obstacle_weight_; // high cost is bad
            output_cost.path = path_map_[index_x + index_y * common_map_.map_msg_.info.width] * vel_path_weight_;
        }
    } else {
        //ROS_WARN("Local planner path is out of map");
        output_cost.total = punitive_cost;
        valid_vel = false;
    }

    if (!global_path_.poses.empty()) {
        output_cost.goal = std::sqrt(std::pow(global_path_.poses[0].pose.position.x - predict_x, 2) + 
                                std::pow(global_path_.poses[0].pose.position.y - predict_y, 2)) * vel_goal_weight_;
    }
    if (valid_vel){ 
        output_cost.total = output_cost.obstacle + output_cost.goal;
        if (use_path_cost_ == true) {
            output_cost.total += output_cost.path;
        }
    }
}

void POCVLocalPlannerROS::findAngularCmdVel(geometry_msgs::Twist& predict_cmd_vel){
    predict_cmd_vel.linear.x = 0.0;
    predict_cmd_vel.linear.y = 0.0;
    double min_ang_vz_from_ang_az = 1.0*-max_abs_angular_acceleration_*time_step_ + last_cmd_vel_.angular.z;
    double max_ang_vz_from_ang_az = 1.0*max_abs_angular_acceleration_*time_step_ + last_cmd_vel_.angular.z;
    double min_ang_vz = std::max(min_ang_vz_from_ang_az, min_angular_velocity_);
    double max_ang_vz = std::min(max_ang_vz_from_ang_az, max_angular_velocity_);
    double ang_vz = delta_yaw_from_current_base_ / time_step_;
    predict_cmd_vel.angular.z = std::min(std::max(ang_vz, min_ang_vz), max_ang_vz);
    //ROS_INFO("predict_cmd_vel.angular.z = %f", predict_cmd_vel.angular.z);
}

void POCVLocalPlannerROS::publishSimulatedValues() {
    // sim_vel_cloud_ptr_->is_dense = true;
    // sensor_msgs::PointCloud2 sim_vel_cloud_msg;
    // pcl::toROSMsg(*sim_vel_cloud_ptr_, sim_vel_cloud_msg);
    // sim_vel_cloud_msg.header.frame_id = param_map_frame_;
    // sim_vel_cloud_msg.header.stamp = ros::Time::now();
    // sim_vels_pub_.publish(sim_vel_cloud_msg);

    sim_base_cloud_ptr_->is_dense = true;
    sensor_msgs::PointCloud2 sim_base_cloud_msg;
    pcl::toROSMsg(*sim_base_cloud_ptr_, sim_base_cloud_msg);
    sim_base_cloud_msg.header.frame_id = param_map_frame_;
    sim_base_cloud_msg.header.stamp = ros::Time::now();
    sim_poses_pub_.publish(sim_base_cloud_msg);
}

void POCVLocalPlannerROS::timerPublish(const ros::TimerEvent& event)
{
    if (is_initial_) {
        publishSimulatedValues();
    }
}

void POCVLocalPlannerROS::updatePlanner() {
    double frequency = 10.0;
    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::Time start_time = ros::Time::now();
        ros::spinOnce(); // call callback msg
        time_step_ = (start_time_ - last_time_).toSec();
        if (time_step_ < 1.0/frequency) {
            time_step_ = 1.0/frequency;
        }
        tf2::Quaternion delta_q, inv_q_base_in_map, q_goal; 
        switch (state_) {
            case WAIT_FOR_GOAL:
                if (ros::Time::now() - last_time_achieve_goal_ > ros::Duration(10.0)) {
                    ROS_INFO("Waiting for goal ...");
                    last_time_achieve_goal_ = ros::Time::now();
                    // change state from callback global path msg
                }
                break;

            case Heading_To_Goal:
                tf2::convert(goal_.pose.orientation, q_goal);
                tf2::convert(base_in_map_tf_msg_.transform.rotation, inv_q_base_in_map);
                inv_q_base_in_map[3] = -inv_q_base_in_map[3];
                delta_q = q_goal*inv_q_base_in_map;
                delta_q.normalize();
                delta_yaw_from_current_base_ = tf2::getYaw(delta_q);
                if (std::abs(delta_yaw_from_current_base_) > 0.05) {
                    findAngularCmdVel(predict_cmd_vel_);
                    cmd_vel_pub_.publish(predict_cmd_vel_);
                } else {
                    cmd_vel_pub_.publish(geometry_msgs::Twist());
                    ROS_INFO("\033[1;32m Successfully rotate heading to goal \033[0m");
                    state_ = WAIT_FOR_GOAL;
                }
                break;

            case ACHIEVE_GOAL:
                state_ = NEAR_GOAL ;
                if (!global_path_.poses.empty()) {
                    if (std::abs(goal_.pose.position.x - base_in_map_tf_msg_.transform.translation.x) < goal_tolerance_ &&
                            std::abs(goal_.pose.position.y - base_in_map_tf_msg_.transform.translation.y) < goal_tolerance_) {
                        ROS_INFO("\033[1;32m Achieve goal \033[0m");
                        cmd_vel_pub_.publish(geometry_msgs::Twist());
                        while (!global_path_.poses.empty()){
                            global_path_.poses.erase(global_path_.poses.begin()); // delete repeat goal that is same goal to avoid time up reach goal
                        }
                        global_path_.poses.clear();
                        local_path_.poses.clear();
                        state_ = Heading_To_Goal;
                        use_path_cost_ = true;
                        // sim_time_step_ = temp_sim_time_step_; // reset param
                        // temp_sim_time_step_ = -0.1;
                        use_heading_to_local_point_ = true;
                        last_time_achieve_goal_ = ros::Time::now() - ros::Duration(7.0);
                    }
                }
                break;

            case NEAR_GOAL:
                state_ = ACHIEVE_GLOBAL_POINT;
                if (std::abs(goal_.pose.position.x - base_in_map_tf_msg_.transform.translation.x) < 0.5 &&
                        std::abs(goal_.pose.position.y - base_in_map_tf_msg_.transform.translation.y) < 0.5) {
                    //ROS_INFO("\033[1;35m Near goal \033[0m");
                    state_ = TIME_UP_GLOBAL_POINT;
                    use_path_cost_ = false;
                    use_heading_to_local_point_ = false;
                    // if (temp_sim_time_step_ < 0.0) {
                    //     temp_sim_time_step_ = sim_time_step_; 
                    // }
                    // sim_time_step_ = 0.5; // adjust param
                    break;
                }

            case ACHIEVE_GLOBAL_POINT:
                state_ = TIME_UP_GLOBAL_POINT;
                if (!global_path_.poses.empty()) {
                    if (std::abs(global_path_.poses[0].pose.position.x - base_in_map_tf_msg_.transform.translation.x) < global_point_tolerance_ &&
                            std::abs(global_path_.poses[0].pose.position.y - base_in_map_tf_msg_.transform.translation.y) < global_point_tolerance_) {
                        //ROS_INFO("Achieve global point");
                        cmd_vel_pub_.publish(geometry_msgs::Twist());
                        global_path_.poses.erase(global_path_.poses.begin());
                        state_ = Heading_To_GLOBAL_POINT;
                        receive_path_map_ = false;
                        last_time_local_goal_ = ros::Time::now();
                    }
                }
                break;
            
            case TIME_UP_GLOBAL_POINT:
                state_ = PATH_INTERSECT_OBSTACLE;
                if (!global_path_.poses.empty()) {
                    if (ros::Time::now() - last_time_local_goal_ > ros::Duration(20.0)){
                        ROS_WARN("Time up to reach global point");
                        cmd_vel_pub_.publish(geometry_msgs::Twist());
                        global_path_.poses.erase(global_path_.poses.begin()); // // time out to achieve local goal, then skip local goal
                        receive_path_map_ = false;
                        state_ = PLAN_PATH;
                        last_time_local_goal_ = ros::Time::now();
                    }
                } else {
                    state_ = WAIT_FOR_GOAL;
                }
                break;  

            case Heading_To_GLOBAL_POINT:
                state_ = PLAN_PATH;
                if (!global_path_.poses.empty()) {
                    double heading_global_yaw = std::atan2(global_path_.poses[0].pose.position.y - base_in_map_tf_msg_.transform.translation.y,
                                                                global_path_.poses[0].pose.position.x - base_in_map_tf_msg_.transform.translation.x);
                    q_global_point_.setRPY(0.0, 0.0, heading_global_yaw);
                    tf2::convert(base_in_map_tf_msg_.transform.rotation, inv_q_base_in_map);
                    inv_q_base_in_map[3] = -inv_q_base_in_map[3];
                    delta_q = q_global_point_*inv_q_base_in_map;
                    delta_q.normalize();
                    delta_yaw_from_current_base_ = tf2::getYaw(delta_q);
                    if (std::abs(delta_yaw_from_current_base_) > 0.1){
                        temp_min_angular_velocity_ = min_angular_velocity_;
                        temp_max_angular_velocity_ = max_angular_velocity_;
                        min_angular_velocity_ = -1.0;
                        max_angular_velocity_ = 1.0;
                        findAngularCmdVel(predict_cmd_vel_);
                        cmd_vel_pub_.publish(predict_cmd_vel_);
                        min_angular_velocity_ = temp_min_angular_velocity_;
                        max_angular_velocity_ = temp_max_angular_velocity_;
                        //cmd_ang_vz_ = predict_cmd_vel_.angular.z;
                        state_ = Heading_To_GLOBAL_POINT;
                    }
                }
                break; 

            case PLAN_PATH:
                state_ = PATH_INTERSECT_OBSTACLE;
                if (!global_path_.poses.empty()) {
                    ROS_INFO("Plan local path");
                    cmd_vel_pub_.publish(geometry_msgs::Twist());
                    receive_path_map_ = createPathMap();
                    if (!receive_path_map_) {
                        state_ = PLAN_PATH;
                    }
                } else {
                    state_ = WAIT_FOR_GOAL;
                }
                break;

            case PATH_INTERSECT_OBSTACLE:
                state_ = Heading_To_LOCAL_POINT;
                if (doPathIntersectObstacle() && ros::Time::now() - last_time_intersect_ > ros::Duration(1.0)){
                    ROS_WARN("Path intersect the obstacle");
                    cmd_vel_pub_.publish(geometry_msgs::Twist());
                    state_ = PLAN_PATH;
                    receive_path_map_ = false;
                    last_time_intersect_ = ros::Time::now();
                }
                break; 

            case Heading_To_LOCAL_POINT:
                state_ = SIMULATE_VELOCITY;
                if (!heading_local_path_.poses.empty()) {
                    if (std::abs(heading_local_path_.poses[0].pose.position.x - base_in_map_tf_msg_.transform.translation.x) < 0.3 &&
                            std::abs(heading_local_path_.poses[0].pose.position.y - base_in_map_tf_msg_.transform.translation.y) < 0.3) {
                        cmd_vel_pub_.publish(geometry_msgs::Twist());
                        heading_local_path_.poses.erase(heading_local_path_.poses.begin());
                    }
                    if (use_heading_to_local_point_) {
                        double heading_local_yaw = std::atan2(heading_local_path_.poses[0].pose.position.y - base_in_map_tf_msg_.transform.translation.y,
                                                                    heading_local_path_.poses[0].pose.position.x - base_in_map_tf_msg_.transform.translation.x);
                        q_local_point_.setRPY(0.0,0.0,heading_local_yaw);
                        tf2::convert(base_in_map_tf_msg_.transform.rotation, inv_q_base_in_map);
                        inv_q_base_in_map[3] = -inv_q_base_in_map[3];
                        delta_q = q_local_point_*inv_q_base_in_map;
                        delta_q.normalize();
                        delta_yaw_from_current_base_ = tf2::getYaw(delta_q);
                        //ROS_INFO("delta_yaw_from_current_base_ = %f", delta_yaw_from_current_base_);
                        if (std::abs(delta_yaw_from_current_base_) > 0.3) {
                            local_point_pub_.publish(heading_local_path_.poses[0]);
                            findAngularCmdVel(predict_cmd_vel_);
                            if (wait_heading_to_local_point_){
                                cmd_vel_pub_.publish(predict_cmd_vel_);
                                state_ = Heading_To_LOCAL_POINT;
                            } else {
                                cmd_ang_vz_ = predict_cmd_vel_.angular.z;
                            }
                        }
                    }
                }
                break;  

            case SIMULATE_VELOCITY:
                state_ = ACHIEVE_GOAL;
                if (!receive_path_map_) {
                    state_ = PLAN_PATH;
                } else {
                    global_point_pub_.publish(global_path_.poses[0]);
                    start_time_ = ros::Time::now();
                    bool found_velocity = findBestVelocity(predict_cmd_vel_);
                    last_time_ = ros::Time::now();
                    //ROS_WARN("Time execution to find best velocity : %f seconds", (last_time_ - start_time_).toSec());
                    if (found_velocity) {
                        if (wait_heading_to_local_point_) {
                            predict_cmd_vel_.angular.z = 0.0;
                        } else {
                            predict_cmd_vel_.angular.z = cmd_ang_vz_;
                        }
                        cmd_vel_pub_.publish(predict_cmd_vel_);
                        //ROS_WARN("Time execution of local planner simulating velocity : %f seconds", (ros::Time::now() - start_time).toSec());     
                        rate.sleep();
                    } else{
                        cmd_vel_pub_.publish(geometry_msgs::Twist());
                    }
                }
                break;
        }
        last_cmd_vel_ = predict_cmd_vel_;
    }
}

void POCVLocalPlannerROS::run(){
    ros::Duration(0.5).sleep();
    ros::Rate rate(50);
    while (!receive_laser_msg_ || !receive_map_msg_ || !receive_odom_msg_) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Local Planner Ready to plan");
    is_initial_ = true;
    updatePlanner();
}

void POCVLocalPlannerROS::handleLookupTransform(const std::string& target_frame, const std::string& source_frame, 
                            geometry_msgs::TransformStamped& output_transform_stamped) {
    try {
        ros::Rate r(100);
        ros::Time last_time = ros::Time::now();
        while(!tf_buffer_.canTransform(target_frame, source_frame, ros::Time(0))) {
            if ((ros::Time::now() - last_time) > ros::Duration(0.1)){
                ROS_WARN("Local planner Time up 0.1 sec. Transform from %s to %s not available.", source_frame.c_str(),target_frame.c_str());
                last_time = ros::Time::now();
            }
            r.sleep();
        } 
        //ros::Time start_time = ros::Time::now();
        output_transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
        //ROS_INFO("Time execution to lookup Transform : %f seconds", (ros::Time::now() - start_time).toSec());
    } catch (const tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }
}

void POCVLocalPlannerROS::publish_path(nav_msgs::Path& path_msg) {
    path_msg.header.frame_id = param_map_frame_;
    path_msg.header.stamp = ros::Time::now();
    local_path_pub_.publish(path_msg);
}

