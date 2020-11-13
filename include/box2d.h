/*
 * box2d.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef BOX2D_H
#define BOX2D_H

#include <vector>
#include <limits>

#include "vector_2d.h"
#include "line_segment2d.h"


namespace math{


class Box2d{
    public:
        Box2d() = default;

        /*
         * @brief Constructor a box with center, yaw, lenght and width
         */
        Box2d(const Vector2d &center, const double heading, 
              const double length, const double width);

        /****** the externel interface ******/
        /*
         * @brief the function to return the center of box
         * @return the center point of box
         */
        const Vector2d &getCenter() const { return _center; }

        /*
         * @brief get the x-coordinate of the center of the box
         * @return the x-coordinate of the center of the box
         */
        double getCenterX() const { return _center.getX(); }

        /*
         * @brief get the y-coordinate of the center of the box
         * @return the y-coordinate of the center of the box
         */
        double getCenterY() const { return _center.getY(); }

        /*
         * @brief get the heading angle of the box
         * @brief return the heading of the box
         */
        double getHeading() const { return _heading; }

        /*
         * @brief Get the cosine of the heading angle
         * @return The cosine of the heading angle
         */
        double getCosHeading() const { return _cos_heading; }

        /*
         * @brief Get the sine of the heading angle
         * @return The sine of the heading angle
         */
        double getSinHeading() const { return _sin_heading; }

        /*
         * @brief get the lenght of the box
         * @return the lenght of the box
         */
        double getLength() const { return _length; } 

        /*
         * @brief get the width of the box
         * @return the width of the box
         */
        double getWidth() const { return _width; }

        /*
         * @brief get the half lenght of the box
         * @return the half lenght of the box
         */
        double getHalfLength() const { return _half_length; } 

        /*
         * @brief get the half width of the box
         * @return the half width of the box
         */
        double getHalfWidth() const { return _half_width; }

        void InitCorners();
        /**
         * @brief get the corners
         */
        double getMax_x() const { return _max_x; }
        double getMin_x() const { return _min_x; }
        double getMax_y() const { return _max_y; }
        double getMin_y() const { return _min_y; }


        /****** the relationship betwen point and box ******/
        /*
         * @brief Test the point whether in the box
         * @param point: A point which be tested whether in the box
         * @return True:if the point is contained in the box;
         */
        bool IsPointIn(const Vector2d &point) const;

        /*
         * @brief Test the point whether is on the boundary of the box
         * @param point: The point which be tested whether is on the boundary of the
         * box
         * @return True: if the point is on the boundary on the box
         */
        bool IsPointOnBoundary(const Vector2d &point) const;

        /**
         * @brief Determines whether this box overlaps a given line segment
         * @param line_segment: The line-segment
         * @return True: if they overlap
         */
        bool HasOverlap(const LineSegment2d &line_segment) const;

        /**
         * @brief Determines whether the two boxs overlap
         * @param box: The other box 
         * @return True: if they overlap
         */
        bool HasOverlap(const Box2d &box) const;

        /*
         * @brief Calculate the distance between the box and the point
         * @param point: the point whose distanc to the box we wish to conpute
         * @return A distance
         */
        double DistanceTo(const Vector2d &point) const;

        /*
         * @brief Calculate the distance between the box and a given line
         * segment
         * @param line_segment: the line segment whose distanc to the box we wish to conpute
         * @return A distance
         */
        double DistanceTo(const LineSegment2d &line_segment) const;

        /*
         * @brief Calculate the distance between two box 
         * @param box: the box whose distanc to the other box we wish to conpute
         * @return A distance
         */
        double DistanceTo(const Box2d &box) const;

         
    private:

        Vector2d _center;
        double   _heading;
        double   _cos_heading;
        double   _sin_heading;

        double   _length;
        double   _width;
        double   _half_length;
        double   _half_width;
        
        std::vector<Vector2d> _corners;

        double _max_x = std::numeric_limits<double>::lowest();
        double _min_x = std::numeric_limits<double>::max();
        double _max_y = std::numeric_limits<double>::lowest();
        double _min_y = std::numeric_limits<double>::max();
};
}

#endif /* !BOX2D_H */
