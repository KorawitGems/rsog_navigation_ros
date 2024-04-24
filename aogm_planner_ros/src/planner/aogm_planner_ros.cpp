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

#include <aogm_planner_ros/planner/aogm_planner_ros.hpp>

double AOGMPlannerROS::calculateDistance(const int& current_x, const int& current_y,
                        const int& goal_x, const int& goal_y) {
    return std::sqrt(std::pow(goal_x - current_x, 2) + std::pow(goal_y - current_y, 2));
}

bool AOGMPlannerROS::plan_path(AOGMCommonMap& common_map, const std::vector<int>& occupancy_map, 
                                const std::vector<double>& obstacle_map, const double& goal_cost_weight, 
                                const double& move_cost_weight, nav_msgs::Path& output_path) {

    expanded_nodes_.clear();
    visited_nodes_.clear();
    node_ptr start_node = std::make_shared<Node>(start_index_);
    node_ptr goal_node = std::make_shared<Node>(goal_index_);

    expanded_nodes_.push(start_node);

    geometry_msgs::PoseStamped result_pose;
    result_pose.header.frame_id = "map";
    result_pose.pose.orientation.w = 1.0;

    bool found_path = false;
    while (!expanded_nodes_.empty()) {
        node_ptr current_node = expanded_nodes_.top();
        expanded_nodes_.pop();
        visited_nodes_.insert(current_node->map_index.y * common_map.map_msg_.info.width + current_node->map_index.x);

        double distance_to_goal = calculateDistance(current_node->map_index.x, current_node->map_index.y, 
                                                    goal_node->map_index.x, goal_node->map_index.y);
        if (distance_to_goal < 0.1) { // goal cell is same current cell
            while (current_node->parent != nullptr) {
                common_map.mapIndexToPosition(current_node->map_index.x, current_node->map_index.y,
                                        result_pose.pose.position.x, result_pose.pose.position.y);
                output_path.poses.emplace_back(result_pose);
                current_node = current_node->parent;
            }
            std::reverse(output_path.poses.begin(), output_path.poses.end());
            found_path = true;
            return found_path;
        }

        std::vector<node_ptr> neighbor_nodes;
        getNeighbors(common_map, occupancy_map, current_node, neighbor_nodes);
        for (auto& neighbor_node : neighbor_nodes) {
            if (visited_nodes_.find(neighbor_node->map_index.y * common_map.map_msg_.info.width + neighbor_node->map_index.x) != visited_nodes_.end())
                continue;
            neighbor_node->parent = current_node;
            neighbor_node->move_cost = current_node->move_cost + 1.0;
            neighbor_node->goal_cost = calculateDistance(neighbor_node->map_index.x, neighbor_node->map_index.y, goal_index_.x, goal_index_.y);
            neighbor_node->total_cost = neighbor_node->move_cost * move_cost_weight + neighbor_node->goal_cost * goal_cost_weight +
                                        obstacle_map[neighbor_node->map_index.y * common_map.map_msg_.info.width + neighbor_node->map_index.x];
            expanded_nodes_.push(neighbor_node);
            visited_nodes_.insert(neighbor_node->map_index.y * common_map.map_msg_.info.width + neighbor_node->map_index.x);
        }
    }

    return found_path;
}

void AOGMPlannerROS::getNeighbors(AOGMCommonMap& common_map, const std::vector<int>& occupancy_map, 
                            const node_ptr& current_node, std::vector<node_ptr>& neighbor_nodes) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            int new_x = current_node->map_index.x + dx;
            int new_y = current_node->map_index.y + dy;
            
            if (common_map.isInMap(new_x, new_y) && occupancy_map[new_y * common_map.map_msg_.info.width + new_x] != 100) 
            {
                node_ptr neighbor_node = std::make_shared<Node>(MapIndex(new_x, new_y));
                neighbor_nodes.emplace_back(neighbor_node);
            }
        }
    }
}
