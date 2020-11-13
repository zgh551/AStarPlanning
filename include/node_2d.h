/*
 * node_2d.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef NODE_2D_H
#define NODE_2D_H

#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <utility>

#include <ros/ros.h>

namespace planning{

class Node2d
{
    public:
    Node2d(const double x, const double y, const double xy_resolution, 
           const std::vector<double> &XYbounds)
    {
        // XYBound with x_min, x_max, y_min, y_max
        _x = x;
        _y = y;
        _grid_x = static_cast<int16_t>( (x - XYbounds[0]) / xy_resolution );
        _grid_y = static_cast<int16_t>( (y - XYbounds[2]) / xy_resolution );
    }

    // external interface
    // Set function
    void setMoveCost(const double move_cost) {
        _move_cost = move_cost;
        _cost = move_cost + _heuristic;
    }

    void setHeuristic(const double heuristic) {
        _heuristic = heuristic;
        _cost = _move_cost + heuristic;
    }

    void setCost(const double cost) { _cost = cost; }
    void setParentNode(const std::shared_ptr<Node2d> parent_node) { _parent_node = parent_node; }

    // Get function
    double getX(void) const { return _x; }
    double getY(void) const { return _y; }

    double getGridX(void) const { return _grid_x; }
    double getGridY(void) const { return _grid_y; }
    
    double getMoveCost(void) const { return _move_cost; }
    double getHeuristic(void) const { return _heuristic; }
    double getCost(void) const { return _cost; }

    // get the string index of node
    const std::string& getIndex(void) const { return _string_index; }
    // get the parent node
    std::shared_ptr<Node2d> getParentNode(void) const { return _parent_node; }

    static std::string CalculateIndex(const double x, const double y,
                                      const double xy_resolution,
                                      const std::vector<double> &XYbounds)
    {
        // XYBound with min_x, max_x, min_y, max_y
        int16_t grid_x = static_cast<int16_t>( (x - XYbounds[0]) / xy_resolution );
        int16_t grid_y = static_cast<int16_t>( (y - XYbounds[2]) / xy_resolution );
        return ComputeStringIndex(grid_x, grid_y);
    }

    // the operator of ==
    bool operator==(const Node2d &right) const {
        return right.getIndex() == _string_index;
    }
    private:
    static std::string ComputeStringIndex(int16_t x_grid, int16_t y_grid)
    {
        return std::to_string(x_grid) + "_" + std::to_string(y_grid);
    }

    // the continuous position of node
    double _x = 0.0;
    double _y = 0.0;
    // the number of grid in x and y axis
    int16_t _grid_x = 0;
    int16_t _grid_y = 0;
    // the cost of node
    double _move_cost = 0.0;
    double _heuristic = 0.0;
    double _cost = 0.0;
    // the parent node of current node
    std::shared_ptr<Node2d> _parent_node = nullptr;
    // the string of grid index
    std::string _string_index;
};

struct GridAStarResult {
    std::vector<double> x;
    std::vector<double> y;
    double move_cost = 0.0;
};
}

#endif /* !NODE_2D_H */
