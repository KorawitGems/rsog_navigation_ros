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

#include <apmc_localization_ros/apmc_localization_ros.hpp>

APMCLocalizationROS::APMCLocalizationROS() : tf_listener_(tf_buffer_), pnh_("~")
{
    pnh_.param<std::string>("map_frame", param_map_frame_, "map");
    pnh_.param<std::string>("odom_frame", param_odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", param_base_frame_, "base_footprint");
    pnh_.param<bool>("publish_tf", publish_tf_, true);
    pnh_.param<double>("linear_update", param_linear_update_, 2.0);
    pnh_.param<double>("angular_update", param_angular_update_, 2.0);
    pnh_.param<double>("sample_linear_resolution", param_sample_linear_resolution_, 0.05);
    pnh_.param<double>("sample_linear_size", param_sample_linear_size_, 0.5);
    pnh_.param<double>("resample_linear_resolution", param_resample_linear_resolution_, 0.01);
    pnh_.param<double>("resample_linear_size", param_resample_linear_size_, 0.05);
    pnh_.param<double>("sample_angular_resolution", param_sample_angular_resolution_, 2.0); // deg
    pnh_.param<double>("sample_angular_size", param_sample_angular_size_, 40.0); // deg
    pnh_.param<double>("resample_angular_resolution", param_resample_angular_resolution_, 0.5); // deg
    pnh_.param<double>("resample_angular_size", param_resample_angular_size_, 2.0); // deg
    pnh_.param<double>("hit_weight", param_hit_weight_, 0.8);
    pnh_.param<double>("rand_weight", param_rand_weight_, 0.2);
    pnh_.param<double>("sigma_hit", param_sigma_hit_, 0.01);
    pnh_.param<int>("laser_step", param_laser_step_, 1);
    pnh_.param<double>("inflated_occupied_radius", inflated_occupied_radius_, 0.1);
    pnh_.param<bool>("debug/time_execution", debug_time_execution_, false);
    receive_map_msg_ = false;
    receive_laser_msg_ = false;
    receive_odom_msg_ = false;
    is_initial_ = false;

    map_sub_ = nh_.subscribe("/map", 1, &APMCLocalizationROS::mapCallback, this);
    laser_sub_ = nh_.subscribe("/scan", 1, &APMCLocalizationROS::laserCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &APMCLocalizationROS::odomCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &APMCLocalizationROS::initialPoseCallback, this);
    
    apmc_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/apmc_pose", 1);
    odom_in_map_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/odom_in_map", 1);
    loop_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &APMCLocalizationROS::publishTfTimer, this);
}

APMCLocalizationROS::~APMCLocalizationROS(){}

void APMCLocalizationROS::initializeNode() {
    ros::Rate r(10); // wait to receive msg
    while (!(receive_map_msg_ && receive_laser_msg_ && receive_odom_msg_)) {
        ros::spinOnce();
        r.sleep();
    }
    particle_in_map_msg_.header.frame_id = param_map_frame_;

    current_odom_msg_.pose.pose.orientation.w = 1.0;
    last_robot_pose_ = current_odom_msg_.pose.pose;

    odom_in_map_tf_msg_.header.frame_id = param_map_frame_;
    odom_in_map_tf_msg_.child_frame_id = param_odom_frame_;
    odom_in_map_tf_msg_.transform.rotation.w = 1.0;
    odom_in_map_tf_msg_.header.stamp = ros::Time::now();
    
    initial_particle_pose_cov_.pose.pose.orientation.w = 1.0;
    initial_particle_pose_cov_.header.stamp = ros::Time::now();
    initial_particle_pose_cov_.header.frame_id = param_map_frame_;

    point_in_base_vector_.resize(std::ceil(laser_msg_.ranges.size()/param_laser_step_));
}

void APMCLocalizationROS::initializeProcess()
{
    randomProbability(laser_msg_.range_max, p_rand_);
    handleLookupTransform(param_base_frame_, laser_msg_.header.frame_id, laser_in_base_tf_msg_);
    is_initial_ = true;
    ROS_INFO("Initialized apmc localization");
}

void APMCLocalizationROS::run()
{
    initializeNode();
    initializeProcess();
    updateLocalization();
}

void APMCLocalizationROS::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) { 
    if (receive_map_msg_){
        return;
    }
    common_map_.map_msg_ = *msg;
    grid_map_.resize(msg->info.width*msg->info.height);
    int index_occupied_radius = std::floor(inflated_occupied_radius_ / msg->info.resolution);
    for (int i = 0; i < msg->info.width; ++i) {
        for (int j = 0; j < msg->info.height; ++j) {
            grid_map_[j * msg->info.width + i] = msg->data[j * msg->info.width + i];
        }
    } // finish copy before apply inflated radius
    for (int i = 0; i < msg->info.width; ++i) {
        for (int j = 0; j < msg->info.height; ++j) {
            if (msg->data[j * msg->info.width + i] == 100) {
                for (int dx = -index_occupied_radius; dx <= index_occupied_radius; ++dx) {
                    for (int dy = -index_occupied_radius; dy <= index_occupied_radius; ++dy) {
                        int x = i + dx;
                        int y = j + dy;
                        if (common_map_.isInMap(x,y)) {
                            grid_map_[x + y * msg->info.width] = 100;
                        }
                    }
                }
            }
        }
    }
    receive_map_msg_ = true;
}

void APMCLocalizationROS::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    odom_msg_ = current_odom_msg_; // odom msg when receive laser msg
    laser_msg_ = *msg;
    point_in_base_vector_.clear();
    for (size_t i = 0; i < laser_msg_.ranges.size(); i += param_laser_step_) {
        if (!std::isnan(laser_msg_.ranges[i]) && !std::isinf(laser_msg_.ranges[i]) && laser_msg_.ranges[i] < laser_msg_.range_max && laser_msg_.ranges[i] > laser_msg_.range_min) {
            double angle = laser_msg_.angle_min + i * laser_msg_.angle_increment;
            geometry_msgs::TransformStamped point_in_laser_tf_msg;
            point_in_laser_tf_msg.transform.translation.x = laser_msg_.ranges[i] * std::cos(angle);
            point_in_laser_tf_msg.transform.translation.y = laser_msg_.ranges[i] * std::sin(angle);
            point_in_laser_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.0));
            tf2::Transform point_in_laser_tf2, laser_in_base_tf2;
            tf2::convert(point_in_laser_tf_msg.transform, point_in_laser_tf2); 
            tf2::convert(laser_in_base_tf_msg_.transform, laser_in_base_tf2); 
            tf2::Transform point_in_base_tf2 = laser_in_base_tf2 * point_in_laser_tf2;
            point_in_base_vector_.emplace_back(point_in_base_tf2);
        }
    }
    if (is_initial_) {
        tf2::Transform base_in_odom_tf2;
        tf2::convert(odom_msg_.pose.pose, base_in_odom_tf2);
        tf2::Transform odom_in_map_tf2;
        tf2::convert(odom_in_map_tf_msg_.transform, odom_in_map_tf2); // convert can not use stamped
        tf2::Transform base_in_map_tf2 = odom_in_map_tf2 * base_in_odom_tf2;
        initial_particle_pose_cov_.pose.pose.position.x = base_in_map_tf2.getOrigin().x();
        initial_particle_pose_cov_.pose.pose.position.y = base_in_map_tf2.getOrigin().y();
        initial_particle_pose_cov_.pose.pose.orientation.x = base_in_map_tf2.getRotation().x();
        initial_particle_pose_cov_.pose.pose.orientation.y = base_in_map_tf2.getRotation().y();
        initial_particle_pose_cov_.pose.pose.orientation.z = base_in_map_tf2.getRotation().z();
        initial_particle_pose_cov_.pose.pose.orientation.w = base_in_map_tf2.getRotation().w();
    }
    receive_laser_msg_ = true;
}

void APMCLocalizationROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_msg_ = *msg; 
    receive_odom_msg_ = true;
}

void APMCLocalizationROS::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf2::Quaternion quat;
    Particle command_particle;
    command_particle.x = msg->pose.pose.position.x;
    command_particle.y = msg->pose.pose.position.y;
    tf2::convert(msg->pose.pose.orientation, quat);
    command_particle.theta = tf2::getYaw(quat);
    command_particle.weight = 0.0;
    preparePublisher(command_particle);
}

void APMCLocalizationROS::generateSamples(double& linear_size, double& angular_size, 
                                    double& linear_resolution, double& angular_resolution, geometry_msgs::PoseWithCovarianceStamped& initial_particle_pose)
{
    int min_index_linear = -std::round(linear_size/linear_resolution/2);
    int max_index_linear = std::round(linear_size/linear_resolution/2);
    int min_index_angular = -std::round(angular_size/angular_resolution/2);
    int max_index_angular = std::round(angular_size/angular_resolution/2);
    for (int i = min_index_linear; i <= max_index_linear; ++i) {
        for (int j = min_index_linear; j <= max_index_linear; ++j) {
            for (int t = min_index_angular; t <= max_index_angular; ++t) { // max range -3.14 < yaw < 3.14
                Particle particle = Particle();
                particle.x = initial_particle_pose.pose.pose.position.x + i*linear_resolution;
                particle.y = initial_particle_pose.pose.pose.position.y + j*linear_resolution;
                int map_index_x, map_index_y;
                common_map_.positionToMapIndex(particle.x, particle.y, map_index_x, map_index_y);
                if (common_map_.isInMap(map_index_x, map_index_y) && common_map_.isFreeInMap(map_index_x, map_index_y)) {
                    tf2::Quaternion q_predict, q_rot, q_initial_particle;
                    tf2::convert(initial_particle_pose.pose.pose.orientation, q_initial_particle);
                    double delta_theta = t*angular_resolution*M_PI/180;
                    double delta_yaw = std::atan2(std::sin(delta_theta), std::cos(delta_theta));
                    q_rot.setRPY(0.0, 0.0, delta_yaw);
                    q_predict = q_rot*q_initial_particle;
                    q_predict.normalize();
                    particle.theta = tf2::getYaw(q_predict);
                    calculateLikelihoodField(particle);
                }
            }
        }
    }
}

void APMCLocalizationROS::calculateLikelihoodField(Particle& particle)
{
    double p_hit = 0.0;
    tf2::Quaternion q;
    particle.weight = 0.0;
    for (const auto &point_in_base : point_in_base_vector_) {
        particle_in_map_msg_.pose.pose.position.x = particle.x;
        particle_in_map_msg_.pose.pose.position.y = particle.y;
        particle_in_map_msg_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), particle.theta));
        
        tf2::Transform particle_in_map_tf2;
        tf2::convert(particle_in_map_msg_.pose.pose, particle_in_map_tf2);
        tf2::Transform point_in_map_tf2 = particle_in_map_tf2 * point_in_base;
        int map_index_x, map_index_y;
        common_map_.positionToMapIndex(point_in_map_tf2.getOrigin().x(), point_in_map_tf2.getOrigin().y(), map_index_x, map_index_y);
        if (common_map_.isInMap(map_index_x, map_index_y)) {
            if (grid_map_[map_index_x + map_index_y* common_map_.map_msg_.info.width] == 100) {
                double occ_grid_in_global_x, occ_grid_in_global_y;
                common_map_.mapIndexToPosition(map_index_x, map_index_y, occ_grid_in_global_x, occ_grid_in_global_y);
                double z_distance = std::sqrt(std::pow(point_in_map_tf2.getOrigin().x() - occ_grid_in_global_x, 2) + std::pow(point_in_map_tf2.getOrigin().y() - occ_grid_in_global_y, 2));
                gaussianProbability(z_distance, param_sigma_hit_, p_hit);
                if (z_distance > z_distance_tol_) {
                    p_hit = -1.0; // reduce probability due to laser do not match grid map well
                }
            } else {
                p_hit = -3.0; // laser do not align occupancy grid map
            }
        } else {
            particle.weight = -1.0; // reject due to laser out of map
            break;
        }
        particle.weight += param_hit_weight_ * p_hit + param_rand_weight_ * p_rand_;
    }
    //ROS_INFO("particle.weight : %f", max_weight_particle_.weight);
    if (particle.weight > max_weight_particle_.weight) {
        found_particle_ = true;
        max_weight_particle_ = particle;
    }
}

void APMCLocalizationROS::gaussianProbability(const double& z, const double& sigma, double& output_p_hit) {
    output_p_hit = (1.0 / (std::sqrt(2.0 * M_PI) * sigma)) * std::exp(-0.5 * std::pow(z / sigma, 2));
}

void APMCLocalizationROS::randomProbability(const double& z_rand, double& output_p_rand) {
    output_p_rand = 1.0 / z_rand;
}

void APMCLocalizationROS::handleLocalization() {
    if (receive_map_msg_ && receive_laser_msg_ && receive_odom_msg_) {           
        max_weight_particle_ = Particle(); // reset
        max_weight_particle_.x = 0.0;
        max_weight_particle_.y = 0.0;
        max_weight_particle_.theta = 0.0;
        max_weight_particle_.weight = 0.0;
        //ros::Time start_time = ros::Time::now();
        z_distance_tol_ = 0.8*common_map_.map_msg_.info.resolution;
        generateSamples(param_sample_linear_size_, param_sample_angular_size_, param_sample_linear_resolution_, param_sample_angular_resolution_, initial_particle_pose_cov_); // around robot's position
        geometry_msgs::PoseWithCovarianceStamped resample_pose_cov;
        resample_pose_cov.pose.pose.position.x = max_weight_particle_.x;
        resample_pose_cov.pose.pose.position.y = max_weight_particle_.y;
        resample_pose_cov.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), max_weight_particle_.theta));
        z_distance_tol_ = param_resample_linear_resolution_;
        generateSamples(param_resample_linear_size_, param_resample_angular_size_, param_resample_linear_resolution_, param_resample_angular_resolution_, resample_pose_cov); // very near robot's position
        //ROS_WARN("Time execution of generateSamples() : %f seconds", (ros::Time::now() - start_time).toSec());
        if (found_particle_){
            //ROS_INFO("\033[1;32m Successfully found best particle to localize : weight = %f \033[0m", max_weight_particle_.weight);
            preparePublisher(max_weight_particle_);
        }
    }
}

void APMCLocalizationROS::updateLocalization()
{
    ros::Rate rate(100);
    while (ros::ok()) 
    {
        ros::Time start_time = ros::Time::now();
        ros::spinOnce();
        tf2::Quaternion delta_q, odom_q, inv_last_q; 
        tf2::convert(odom_msg_.pose.pose.orientation, odom_q);
        tf2::convert(last_robot_pose_.orientation, inv_last_q);
        inv_last_q[3] = -inv_last_q[3];
        delta_q = odom_q*inv_last_q;
        delta_q.normalize();
        double delta_yaw = tf2::getYaw(delta_q);

        if (std::sqrt(std::pow(odom_msg_.pose.pose.position.x - last_robot_pose_.position.x, 2) +
                std::pow(odom_msg_.pose.pose.position.y - last_robot_pose_.position.y, 2)) > param_linear_update_ ||
                    std::abs(delta_yaw) > param_angular_update_) 
        {
            found_particle_ = false;
            handleLocalization();
            last_robot_pose_ = odom_msg_.pose.pose;
            if (debug_time_execution_) {
                ROS_WARN("Time execution of localization : %f seconds", (ros::Time::now() - start_time).toSec());
            }
        }
        rate.sleep();
    }
}

void APMCLocalizationROS::preparePublisher(const Particle& particle) {
    particle_in_map_msg_.pose.pose.position.x = particle.x;
    particle_in_map_msg_.pose.pose.position.y = particle.y;
    particle_in_map_msg_.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), particle.theta));
    particle_in_map_msg_.header.stamp = ros::Time::now();
    apmc_pose_pub_.publish(particle_in_map_msg_);
    
    tf2::Transform base_in_odom_tf2;
    tf2::convert(odom_msg_.pose.pose, base_in_odom_tf2);
    tf2::Transform particle_in_map_tf2;
    tf2::convert(particle_in_map_msg_.pose.pose, particle_in_map_tf2); // convert can not use stamped
    tf2::Transform odom_in_map_tf2 = particle_in_map_tf2 * base_in_odom_tf2.inverse();
    odom_in_map_tf_msg_.transform = tf2::toMsg(odom_in_map_tf2);
}

void APMCLocalizationROS::publishTfTimer(const ros::TimerEvent& event)
{
    if (is_initial_) {
        odom_in_map_tf_msg_.header.stamp = ros::Time::now();
        if (publish_tf_) {
            tf_broadcaster_.sendTransform(odom_in_map_tf_msg_);
        }
        odom_in_map_pub_.publish(odom_in_map_tf_msg_);
    }
}

void APMCLocalizationROS::handleLookupTransform(const std::string& target_frame, const std::string& source_frame, 
                            geometry_msgs::TransformStamped& output_transform_stamped) {
    try {
        ros::Rate r(100);
        ros::Time last_time = ros::Time::now();
        while(!tf_buffer_.canTransform(target_frame, source_frame, ros::Time(0))) {
            if ((ros::Time::now() - last_time) > ros::Duration(0.1)){
                ROS_WARN("Localization time up 0.1 sec. Transform from %s to %s not available.", source_frame.c_str(), target_frame.c_str());
                return;
            }
            r.sleep();
        } 
        output_transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }
}