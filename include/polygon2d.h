/*
 * polygon2d.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef POLYGON2D_H
#define POLYGON2D_H

#include <string>
#include <vector>

#include "vector_2d.h"
#include "line_segment2d.h"
#include "box2d.h"


namespace math
{
    class Polygon2d
    {
    public:
        /**
         * @brief Empty constructor
         */
        Polygon2d() = default;

        /**
         * @brief Constructor which take a box
         * @param box: The box to construct the polygon.
         */
        explicit Polygon2d(const Box2d &box);

        /**
         * @brief Constructor which takes a vector of points as its vertices.
         * @param points: The points to constructor the polygon.
         */
        explicit Polygon2d(std::vector<Vector2d> points);


        /****** extern interface ******/
        /**
         * @brief Get the vertices of the polygon.
         * @return The vertices of the polygon.
         */
        const std::vector<Vector2d> &getPoints() const { return points_; }

        /**
         * @brief Get the edges of the polygon.
         * @return The edges of the polygon.
         */
        const std::vector<LineSegment2d> &getLineSegments() const { return line_segments_; }

        /**
         * @brief Get the number of vertices of the polygon.
         * @return The number of the vertices of the polygon.
         */
        int16_t getNumPoints() const { return num_point; }

        /**
         * @brief Check if the polygon is convex.
         * @return True: The polygon is convex.
         */
        bool getIsConvex() const { return is_convex_; }

        /**
         * @brief Get the area of the polygon
         * @return The area of the polygon
         */
        double getArea() const { return area_; }

        double getMin_x() const { return min_x_; }
        double getMax_x() const { return max_x_; }
        double getMin_y() const { return min_y_; }
        double getMax_y() const { return max_y_; }
    protected:

        std::vector<Vector2d> points_;
        int16_t num_point;
        std::vector<LineSegment2d> line_segments_;
        bool is_convex_ = false;
        double area_  = 0.0;
        double min_x_ = 0.0;
        double max_x_ = 0.0;
        double min_y_ = 0.0;
        double max_y_ = 0.0;
    };
}

#endif /* !POLYGON2D_H */
