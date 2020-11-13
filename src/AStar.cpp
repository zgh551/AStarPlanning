/*
 * AStar.cpp
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 * 
 * Distributed under terms of the MIT license.
 */

#include "../include/AStar.h"
#include <algorithm>
#include <cstring>
#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace planning
{
    double AStar::EuclidDistance(const double x1, const double y1,
                                 const double x2, const double y2) const
    {
        return hypot(x1 - x2, y1 - y2);
    }

    std::vector<std::shared_ptr<Node2d>> AStar::GenerateNextNodes(std::shared_ptr<Node2d> current_node) 
    {
        const double current_node_x           = current_node->getX();
        const double current_node_y           = current_node->getY();
        const double current_node_move_cost   = current_node->getMoveCost();

        const double diagonal_distance = 1.414;
        std::vector<std::shared_ptr<Node2d>> next_nodes;

        // up
        std::shared_ptr<Node2d> up = std::make_shared<Node2d>(
                                     current_node_x, current_node_y + _xy_grid_resolution,
                                     _xy_grid_resolution, _XYbounds);
        up->setMoveCost(current_node_move_cost + 1.0);

        // up_right
        std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
                                           current_node_x + _xy_grid_resolution, 
                                           current_node_y + _xy_grid_resolution,
                                           _xy_grid_resolution, _XYbounds);
        up_right->setMoveCost(current_node_move_cost + diagonal_distance);

        // right
        std::shared_ptr<Node2d> right = std::make_shared<Node2d>(
                                        current_node_x + _xy_grid_resolution, current_node_y,
                                        _xy_grid_resolution, _XYbounds);
        right->setMoveCost(current_node_move_cost + 1.0);

        // down_right
        std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
                                             current_node_x + _xy_grid_resolution, 
                                             current_node_y - _xy_grid_resolution,
                                             _xy_grid_resolution, _XYbounds);
        down_right->setMoveCost(current_node_move_cost + diagonal_distance);

        // down 
        std::shared_ptr<Node2d> down = std::make_shared<Node2d>(
                                       current_node_x, current_node_y - _xy_grid_resolution,
                                       _xy_grid_resolution, _XYbounds);
        down->setMoveCost(current_node_move_cost + 1.0);

        // down_left
        std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
                                            current_node_x - _xy_grid_resolution, 
                                            current_node_y - _xy_grid_resolution,
                                            _xy_grid_resolution, _XYbounds);
        down_left->setMoveCost(current_node_move_cost + diagonal_distance);

        // left 
        std::shared_ptr<Node2d> left = std::make_shared<Node2d>(
                                       current_node_x - _xy_grid_resolution, current_node_y,
                                       _xy_grid_resolution, _XYbounds);
        left->setMoveCost(current_node_move_cost + 1.0);

        // up_left
        std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
                                          current_node_x - _xy_grid_resolution, 
                                          current_node_y + _xy_grid_resolution,
                                          _xy_grid_resolution, _XYbounds);
        up_left->setMoveCost(current_node_move_cost + diagonal_distance);

        next_nodes.emplace_back(up);
        next_nodes.emplace_back(up_right);
        next_nodes.emplace_back(right);
        next_nodes.emplace_back(down_right);
        next_nodes.emplace_back(down);
        next_nodes.emplace_back(down_left);
        next_nodes.emplace_back(left);
        next_nodes.emplace_back(up_left);

        return next_nodes;
    }
    
    bool AStar::CheckConstraints(std::shared_ptr<Node2d> node)
    {
        const double node_grid_x = node->getGridX();
        const double node_grid_y = node->getGridY();
        if (node_grid_x > _max_grid_x || node_grid_x < 0 ||
            node_grid_y > _max_grid_y || node_grid_y < 0 )
        {
            return false;
        }

        if (_obstacles_linesegments_vector.empty())
        {
            return true;
        }
        
        for (const auto &obstacles_linesegments : _obstacles_linesegments_vector)
        {
            for (const math::LineSegment2d &linesegment : obstacles_linesegments)
            {
                if (linesegment.DistanceTo({node->getGridX(), node->getY()}) < _node_radius)
                {
                    return false;
                }
            }
        }
        return true;
    }
    
    void AStar::LoadGridAStarResult(GridAStarResult *result)
    {
        (*result).move_cost = -_final_node->getMoveCost() * _xy_grid_resolution;

        std::shared_ptr<Node2d> current_node = _final_node;

        std::vector<double> grid_a_x;
        std::vector<double> grid_a_y;
        while (current_node->getParentNode() != nullptr)
        {
            grid_a_x.push_back(current_node->getX() * _xy_grid_resolution + _XYbounds[0]);
            grid_a_y.push_back(current_node->getY() * _xy_grid_resolution + _XYbounds[2]);
            current_node = current_node->getParentNode();
        }
        std::reverse(grid_a_x.begin(), grid_a_x.end());
        std::reverse(grid_a_y.begin(), grid_a_y.end());

        (*result).x = std::move(grid_a_x);
        (*result).y = std::move(grid_a_y);
    }

    bool AStar::GenerateAStarPath(  const double sx, const double sy,
                                    const double ex, const double ey,
                                    const std::vector<double> &XYbounds,
                                    const std::vector<std::vector<math::LineSegment2d>>
                                    &obstacles_linesegments_vector,
                                    GridAStarResult *result)
    {
        // create the priority queue
        std::priority_queue<std::pair<std::string, double>,
                            std::vector<std::pair<std::string, double>>, cmp> open_pq;

        // create the open and close set
        std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
        std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;

        // update the grid map bounds
        _XYbounds = XYbounds;

       // initialize the start and end node
        _start_node = std::make_shared<Node2d>(sx, sy, _xy_grid_resolution, XYbounds);
        _end_node   = std::make_shared<Node2d>(ex, ey, _xy_grid_resolution, XYbounds);

        // initialize the final node
        _final_node = nullptr;

        // update the line segments of the obstacles
        _obstacles_linesegments_vector = obstacles_linesegments_vector;

        // open set initialize with the start node
        open_set.emplace(_start_node->getIndex(), _start_node);

        // priority queue initialize with start node cost
        open_pq.emplace(_start_node->getIndex(), _start_node->getCost());

        // Grid a star begins
        uint32_t explored_node_num = 0;
        while (!open_pq.empty()) // if open priority queue is not empty continue
        {
            // get the lowest cost node's id
            std::string current_id = open_pq.top().first;

            // node pop
            open_pq.pop();

            // get current node
            std::shared_ptr<Node2d> current_node = open_set[current_id];

            // check destination
            if ( *(current_node) == *(_end_node))
            {
                _final_node = current_node;
                break;
            }
            
            // push current node into the close set
            close_set.emplace(current_node->getIndex(), current_node);

            // generate next node
            //std::vector<std::shared_ptr<Node2d>> 
            auto next_nodes = std::move(GenerateNextNodes(current_node));

            for (auto &next_node : next_nodes)
            {
                // the obstacle avoider checker
                if (!CheckConstraints(next_node)){ continue; }

                // the current node whether is in the close set
                if (close_set.find(next_node->getIndex()) != close_set.end()){ continue; }
                
                // the next node is't in the open set
                if (open_set.find(next_node->getIndex()) == open_set.end())
                {
                    explored_node_num++;
                    next_node->setHeuristic(
                            EuclidDistance(next_node->getX(), next_node->getY(),
                                           _end_node->getGridX(), _end_node->getY()));

                    next_node->setParentNode(current_node);

                    open_set.emplace(next_node->getIndex(), next_node);
                    open_pq.emplace(next_node->getIndex(), next_node->getCost());
                }
                else// node in open set
                {
            
                }
            }
        }
        LoadGridAStarResult(result);
        return true;
    }
}



