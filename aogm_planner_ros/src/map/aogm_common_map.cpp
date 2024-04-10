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

#include <aogm_planner_ros/map/aogm_common_map.hpp>

bool AOGMCommonMap::isFreeInMap(const int& map_index_x, const int& map_index_y) {
    if (map_msg_.data[map_index_x + map_index_y*map_msg_.info.width] == 0) {
        return true;
    }
    return false;
}

bool AOGMCommonMap::isInMap(const int& map_index_x, const int& map_index_y) {
    if (map_index_x >= 0 && map_index_x < map_msg_.info.width && map_index_y >= 0 && map_index_y < map_msg_.info.height) {
        return true;
    }
    return false;
}

void AOGMCommonMap::mapIndexToPosition(const int& map_index_x, const int& map_index_y, double& real_position_x, double& real_position_y) {
    real_position_x = map_msg_.info.origin.position.x + map_index_x * map_msg_.info.resolution;
    real_position_y = map_msg_.info.origin.position.y + map_index_y * map_msg_.info.resolution;
}

void AOGMCommonMap::positionToMapIndex(const double& real_position_x, const double& real_position_y, int& map_index_x, int& map_index_y) {
    map_index_x = std::round((real_position_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
    map_index_y = std::round((real_position_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);
}