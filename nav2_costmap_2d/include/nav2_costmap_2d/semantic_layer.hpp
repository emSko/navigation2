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

#ifndef NAV2_COSTMAP_2D__SEMANTIC_LAYER_HPP_
#define NAV2_COSTMAP_2D__SEMANTIC_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

/**
 * @class SemanticLayer
 * @brief Takes in a map generated from SLAM to add costs to costmap
 */
class SemanticLayer : public Layer
{
public:
  /**
    * @brief Semantic Layer constructor
    */
  SemanticLayer() = default;
  /**
    * @brief Semantic Layer destructor
    */
  virtual ~SemanticLayer() = default;

  /**
   * @brief Initialization process of layer on startup
   */
    void onInitialize() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
   void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
   void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Reset this costmap
   */
   void reset() override;

  /**
   * @brief Match the size of the master costmap
   */
   void matchSize() override;

}; 
} // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__SEMANTIC_LAYER_HPP_
