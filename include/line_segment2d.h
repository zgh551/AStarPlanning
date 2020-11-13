/*
 * line_segment2d.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef LINE_SEGMENT2D_H
#define LINE_SEGMENT2D_H

#include <string>

#include "../include/vector_2d.h"

namespace math
{

class LineSegment2d
{
    public:
    /**
     * @brief LineSegment Empty constructor.
     */
    LineSegment2d();

    /**
     * @brief Construct with start point and end point
     * @param start: The start point tof the line segment
     * @param end: The end point tof the line segment
     */
    LineSegment2d(const Vector2d &start, const Vector2d &end);

    /****** the extern interface ******/
    /**
     * @brief Get the start the point 
     * @return The start point of the line segment 
     */
    const Vector2d &getStart() const { return _start; }

    /**
     * @brief Get the end the point 
     * @return The end point of the line segment 
     */
    const Vector2d &getEnd() const { return _end; }

    /**
     * @brief Get the unit direction from the start point to the end point
     * @return The unit direction point
     */
    const Vector2d &getUnitDirection() const { return _unit_direction; }


    /**
     * @brief Get the heading of the line segment
     * @return The heading, which is the angle between unit direction and
     * x-axis.
     */
    double getHeading() const { return _heading; }

    /**
     * @brief Get the cosine of the heading
     * @return The cosine of the heading
     */
    double getCosHeading() const { return _unit_direction.getX(); }

    /**
     * @brief Get the sine of the heading
     * @return The sine of the heading
     */
    double getSinHeading() const { return _unit_direction.getY(); }

    /****** some base function ******/
    /**
    * @brief Get the center point of the line segment
    * @return The center od the line segment
    */
    Vector2d getCenter() const { return (_start + _end) / 2.0; }

    /**
     * @brief Get a new line-sgment with the same start point,but ratated
     * counterclock-wise by the given angle
     * return The rotated line-segment's end-point
     */
    Vector2d rotate(const double angle);

    /**
     * @brief Get the length of the line segment
     * @return The lenght of the line segment
     */
    double getLength() const;

    /**
     * @brief Get the square of the lenght of the line segment
     * @return The square of lenght of the line segment
     */
    double getLengthSqr() const;

    /**
     * @brief Compute the shortest distance from a point on the line segment to
     * a point in 2-D
     * @param point: The Point to compute the distance to.
     * @return The shortest distance from points on the line segment to point.
     */
    double DistanceTo(const Vector2d &point) const;
    private:
    Vector2d _start;
    Vector2d _end;
    Vector2d _unit_direction;
    double   _heading = 0.0;;
    double   _length  = 0.0;

};

}

#endif /* !LINE_SEGMENT2D_H */
