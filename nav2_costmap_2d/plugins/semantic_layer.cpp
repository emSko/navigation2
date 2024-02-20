// Copyright (c) 2024 Magda Skoczeń
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_costmap_2d/semantic_layer.hpp"

namespace nav2_costmap_2d
{

    void onInitialize() {}

   void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) 
    {
        auto u = robot_x+robot_y+robot_yaw+*min_x+*min_y+*max_x+*max_y;
        robot_x = u;
    }


   void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) 
    {
        master_grid.setCost(min_i, min_j, max_i);
        max_i = max_j;
    }


   void reset(){}

   void matchSize(){}

} // namespace nav2_costmap_2d
