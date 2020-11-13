/*
 * AStar.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "node_2d.h"
#include "line_segment2d.h"

namespace planning{

class AStar{
    public:

    virtual ~AStar() = default;

    /**
     * @brief Base on the start point and end point generate a AStar path
     * @param sx: The x-coordinate of start point
     * @param sy: The y-coordinate of start point
     * @param ex: The x-coordinate of end point
     * @param ey: The y-coordinate of end point
     * @param XYbounds: The min and max bounds od the grid map
     * @param obstacles_linesegments_vector: The line segemnt set of the
     * obstacles
     * @return True: if path gennerate success
     */
    bool GenerateAStarPath( const double sx, const double sy,
                            const double ex, const double ey,
                            const std::vector<double> &XYbounds,
                            const std::vector<std::vector<math::LineSegment2d>>
                            &obstacles_linesegments_vector,
                            GridAStarResult *result);

    /**
     * @brief 
     * @param ex: The x-coordinate of end point
     * @param ey: The y-coordinate of end point
     * @param XYbounds: The min and max bounds od the grid map
     * @param obstacles_linesegments_vector: The line segemnt set of the
     * obstacles
     * @return True: if path gennerate success
     */
    bool GenerateDpMap( const double ex, const double ey,
                        const std::vector<double> &XYbounds,
                        const std::vector<std::vector<math::LineSegment2d>>
                        &obstacles_linesegments_vector);
    
    /**
     * @brief
     * @param sx: The x-coordinate of start point
     * @param sy: The y-coordinate of start point
     * @return 
     */
    double CheckDpMap(const double sx, const double sy);

    private:

    /**
     * @brief Compute the Euclid distance between two points.
     * @param x1: The x-coordinate of one point.
     * @param y1: The y-coordinate of one point.
     * @param x2: The x-coordinate of the other point.
     * @param y2: The y-coordinate of the other point.
     * @return The euclid distance
     */
    double EuclidDistance( const double x1, const double y1,
                           const double x2, const double y2) const;

    /**
     * @brief According to current node generate the next neighboring nodes.
     * @param node: The current node.
     * @return The next nodes.
     */
    std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(std::shared_ptr<Node2d> current_node);

    /**
     * @brief Check the node whether is 
     * @param node: The check node
     * @return True: The node to check
     */
    bool CheckConstraints(std::shared_ptr<Node2d> node);

    /**
     * @brief 
     * @param result: The AStar result
     */
    void LoadGridAStarResult(GridAStarResult *result);

    private:

    double _xy_grid_resolution = 0.0;
    double _node_radius = 0.0;
    std::vector<double> _XYbounds;

    double _max_grid_x = 0.0;
    double _max_grid_y = 0.0;

    std::shared_ptr<Node2d> _start_node;
    std::shared_ptr<Node2d> _end_node;
    std::shared_ptr<Node2d> _final_node;
    std::vector<std::vector<math::LineSegment2d>> _obstacles_linesegments_vector;

    std::unordered_map<std::string, std::shared_ptr<Node2d>> _dp_map;

    struct cmp {
        bool operator()(const std::pair<std::string, double> &left,
                        const std::pair<std::string, double> &right) const{
            return left.second >= right.second;            
        }
    };
};
}

#endif /* !ASTAR_H */
