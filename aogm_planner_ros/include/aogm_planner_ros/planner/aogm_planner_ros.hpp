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

#ifndef AOGM_PLANNER_ROS_HPP
#define AOGM_PLANNER_ROS_HPP

#include <aogm_planner_ros/map/aogm_common_map.hpp>

#include <cmath>
#include <vector>
#include <utility>
#include <boost/heap/binomial_heap.hpp>
#include <chrono>
#include <unordered_set>
#include <memory>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

struct Position {
    double x;
    double y;

    Position(double x_pos = 0, double y_pos = 0) : x(x_pos), y(y_pos) {}
};

struct MapIndex {
    int x;
    int y;

    MapIndex(int x_index = 0, int y_index = 0) : x(x_index), y(y_index) {}
};

struct Node {
    Position position;
    MapIndex map_index;
    double move_cost;
    double goal_cost;
    double total_cost;
    std::shared_ptr<Node> parent;

    Node(MapIndex _map_index)
        : map_index(_map_index) {}
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->total_cost > b->total_cost;
    }
};

class AOGMPlannerROS {
private:
    typedef std::shared_ptr<Node> node_ptr;
    typedef boost::heap::binomial_heap<node_ptr, boost::heap::compare<NodeCompare>> low_priority_queue;
    low_priority_queue expanded_nodes_;
    std::unordered_set<int> visited_nodes_;

public:
    std::vector<geometry_msgs::PoseStamped> current_node_pose_array_;
    MapIndex start_index_;
    MapIndex goal_index_;

    double calculateDistance(const int& current_x, const int& current_y,
                            const int& goal_x, const int& goal_y);

    bool plan_path(AOGMCommonMap& common_map, const std::vector<int>& occupancy_map, 
                                const std::vector<double>& obstacle_map, const double& goal_cost_weight, 
                                const double& move_cost_weight, nav_msgs::Path& output_path);

    void getNeighbors(AOGMCommonMap& common_map, const std::vector<int>& occupancy_map, 
                        const node_ptr& current_node, std::vector<node_ptr>& neighbor_nodes);

};

#endif
