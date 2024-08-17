/*
 * @Author: Yuhao Chen
 * @Date: 2024-06-23 10:26:20
 * @LastEditors: Yuhao Chen
 * @LastEditTime: 2024-06-25 11:49:24
 * @Description: Occupancy grid releated functions. xy_index means the grid x, y index. xy_coord means map frame x, y coordinates. array_index means grid.data array index.
 */

#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"

#include <math.h>

namespace occupancy_grid {
    constexpr int OCCUPIED_THRESHOLD = 50;

    int xy_index_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const int i_x, const int i_y);

    int xy_coord_to_array_index(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y);

    std::pair<int, int> array_index_to_xy_index(const nav_msgs::msg::OccupancyGrid& grid, const int i);

    float array_index_to_x_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i);

    float array_index_to_y_coord(const nav_msgs::msg::OccupancyGrid& grid, const int i);

    bool is_xy_coord_occupied(const nav_msgs::msg::OccupancyGrid& grid, const float x, const float y);

    void set_xy_coord_occupied(nav_msgs::msg::OccupancyGrid& grid, const float x, const float y);

    std::vector<int> inflate_cell(nav_msgs::msg::OccupancyGrid &grid, const int i, const float margin, const int val);

    void inflate_map(nav_msgs::msg::OccupancyGrid& grid, const float margin);
}

#endif