/*
 * box2d.cpp
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 * 
 * Distributed under terms of the MIT license.
 */

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "../include/box2d.h"
#include "../include/line_segment2d.h"
#include "../include/math_utils.h"

namespace math 
{

    void Box2d::InitCorners()
    {
        const double dx1 = _half_length * _cos_heading;
        const double dy1 = _half_length * _sin_heading;
        const double dx2 =  _half_width * _sin_heading;
        const double dy2 = -_half_width * _cos_heading;

        _corners.clear();

        _corners.emplace_back(_center.getX() + dx1 + dx2, _center.getY() + dy1 + dy2);
        _corners.emplace_back(_center.getX() + dx1 - dx2, _center.getY() + dy1 - dy2);
        _corners.emplace_back(_center.getX() - dx1 - dx2, _center.getY() - dy1 - dy2);
        _corners.emplace_back(_center.getX() - dx1 + dx2, _center.getY() - dy1 + dy2);

        for (auto &corner : _corners)
        {
            _max_x = std::fmax(corner.getX(), _max_x);
            _min_x = std::fmin(corner.getX(), _min_x);
            _max_y = std::fmax(corner.getY(), _max_y);
            _min_y = std::fmin(corner.getY(), _min_y);
        }
    }

    Box2d::Box2d(const Vector2d &center, const double heading, 
                 const double length, const double width)
        : _center(center),
          _heading(heading),
          _length(length),
          _width(width)
    {
        _cos_heading = cos(heading);
        _sin_heading = sin(heading);
        _half_width  = width / 2.0;
        _half_length = length / 2.0;
        InitCorners();
    }


    bool Box2d::IsPointIn(const Vector2d &point) const
    {
        // calculate the vector from center poit to the test point
        const double cp_x = point.getX() - _center.getX();
        const double cp_y = point.getY() - _center.getY();
        
        // rotate the vector cp to the initialize position
        const double init_cp_x = std::abs(cp_x * _cos_heading + cp_y * _sin_heading);
        const double init_cp_y = std::abs(cp_x * _sin_heading - cp_y * _cos_heading);

        return (init_cp_x <= _half_length + kMathEpsilon) 
            && (init_cp_y <= _half_width  + kMathEpsilon);
    }

    bool Box2d::IsPointOnBoundary(const Vector2d &point) const
    {
        // calculate the vector from center poit to the test point
        const double cp_x = point.getX() - _center.getX();
        const double cp_y = point.getY() - _center.getY();
        
        // rotate the vector cp to the initialize position
        const double init_cp_x = std::abs(cp_x * _cos_heading + cp_y * _sin_heading);
        const double init_cp_y = std::abs(cp_x * _sin_heading - cp_y * _cos_heading);

        return ((std::abs(init_cp_x - _half_length) <= kMathEpsilon) 
            && (init_cp_y <= _half_width + kMathEpsilon))
            || ((std::abs(init_cp_y - _half_width) <= kMathEpsilon)
            && (init_cp_x <= _half_length + kMathEpsilon));
    }

    double Box2d::DistanceTo(const Vector2d &point) const
    {
         // calculate the vector from center poit to the test point
        const double cp_x = point.getX() - _center.getX();
        const double cp_y = point.getY() - _center.getY();

         // rotate the vector cp to the initialize position
        const double d_cp_x = std::abs(cp_x * _cos_heading + cp_y * _sin_heading) - _half_length;
        const double d_cp_y = std::abs(cp_x * _sin_heading - cp_y * _cos_heading) - _half_width;
      
        if (d_cp_x <= 0.0)
        {
            return std::max(0.0, d_cp_y);
        }

        if (d_cp_y <= 0.0)
        {
            return d_cp_x;
        }

        return hypot(d_cp_x, d_cp_y);
    }

    /****** line-segment ******/
    double Vertex2SegmentDistance(  double box_x, double box_y,
                                    double start_x, double start_y,
                                    double end_x, double end_y, double length)
    {
        const double sb_x = box_x - start_x;
        const double sb_y = box_y - start_y;
        const double se_x = end_x - start_x;
        const double se_y = end_y - start_y;

        // vector dot product
        // sb . se = |sb| * |se| * cos(theta)
        const double dot = sb_x * se_x + sb_y * se_y;

        // if the angle which between the vector sb and se is greater or equal than 90
        // degree,then the value of 'dot' is less or equal the zero.
        if (dot <= 0.0)
        {
            return hypot(sb_x, sb_y);
        }

        if (dot >= length * length)
        {
            return hypot(sb_x - se_x, sb_y - se_y);
        }
        return std::abs(sb_x * se_y - sb_y * se_x) / length;
    }

    double Box2d::DistanceTo(const LineSegment2d &line_segment) const
    {
        double min_distance = 0.0;
        // the line-segment whether is a point
        if (line_segment.getLenght() <= kMathEpsilon)
        {
            return IsPointIn(line_segment.getStart());
        }

        // calculate the box corner point
        double box_x = _half_length;
        double box_y = _half_width;

        // calculate the cs vector(center point to start point)
        const double cs_x = line_segment.getStart().getX() - _center.getX();
        const double cs_y = line_segment.getStart().getY() - _center.getY();
        
        // bas eon the heading angle to rotate the cs vector
        double rotate_cs_x =  cs_x * _cos_heading + cs_y * _sin_heading;
        double rotate_cs_y = -cs_x * _sin_heading + cs_y * _cos_heading;

        // base on the xy-coordinate which rotated to determine the point
        // region
        int8_t binary_cs_x = rotate_cs_x >= box_x ? 1 : (rotate_cs_x <= -box_x ? -1 : 0);
        int8_t binary_cs_y = rotate_cs_y >= box_y ? 1 : (rotate_cs_y <= -box_y ? -1 : 0);
        if (binary_cs_x == 0 && binary_cs_y == 0)
        {
            return 0.0;
        }

        // calculate the ce vector(center point to end point)
        const double ce_x = line_segment.getEnd().getX() - _center.getX();
        const double ce_y = line_segment.getEnd().getY() - _center.getY();
        
        // bas eon the heading angle to rotate the ce vector
        double rotate_ce_x =  ce_x * _cos_heading + ce_y * _sin_heading;
        double rotate_ce_y = -ce_x * _sin_heading + ce_y * _cos_heading;

        // base on the xy-coordinate which rotated to determine the point
        // region
        int8_t binary_ce_x = rotate_ce_x >= box_x ? 1 : (rotate_ce_x <= -box_x ? -1 : 0);
        int8_t binary_ce_y = rotate_ce_y >= box_y ? 1 : (rotate_ce_y <= -box_y ? -1 : 0);
        if (binary_ce_x == 0 && binary_ce_y == 0)
        {
            return 0.0;
        }

        // x-coordinate symmetry transformation
        if (binary_cs_x < 0 || (binary_cs_x == 0 && binary_ce_x < 0))
        {
            rotate_cs_x = -rotate_cs_x;
            binary_cs_x = -binary_cs_x;
            rotate_ce_x = -rotate_ce_x;
            binary_ce_x = -binary_ce_x;
        }

        // y-coordinate symmetry transformation
        if (binary_cs_y < 0 || (binary_cs_y == 0 && binary_ce_y < 0))
        {
            rotate_cs_y = -rotate_cs_y;
            binary_cs_y = -binary_cs_y;
            rotate_ce_y = -rotate_ce_y;
            binary_ce_y = -binary_ce_y;
        }

        // xy-coordinate symmetry transformation
        if (binary_cs_x < binary_cs_y || (binary_cs_x == binary_cs_y && binary_ce_x < binary_ce_y))
        {
            std::swap(rotate_cs_x, rotate_cs_y);
            std::swap(binary_cs_x, binary_cs_y);
            std::swap(rotate_ce_x, rotate_ce_y);
            std::swap(binary_ce_x, binary_ce_y);
            std::swap(box_x, box_y);
        }

        if (binary_cs_x == 1 && binary_cs_y == 1)
        {
            switch(3 * binary_ce_x + binary_ce_y)
            {
                // (1, 1)
                case 4:
                    min_distance =  Vertex2SegmentDistance( box_x, box_y, 
                                                            rotate_cs_x, rotate_cs_y, 
                                                            rotate_ce_x, rotate_ce_y, 
                                                            line_segment.getLenght());
                    break;

                // (1, 0)
                case 3:
                    min_distance = (rotate_cs_x > rotate_ce_x) 
                                 ? (rotate_ce_x - box_x)
                                 : Vertex2SegmentDistance( box_x, box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;

                // (1, -1)
                case 2:
                    min_distance = rotate_cs_x > rotate_ce_x 
                                 ? Vertex2SegmentDistance( box_x, -box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght())
                                 : Vertex2SegmentDistance( box_x, box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;
                
                // (0, -1)
                case -1:
                    min_distance = math::CrossProduct({rotate_cs_x, rotate_cs_y}, 
                                                      {rotate_ce_x, rotate_ce_y},
                                                      {box_x, -box_y}) >= 0.0
                                 ? 0.0 
                                 : Vertex2SegmentDistance( box_x, -box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;

                // (-1, -1)
                case -4:
                    min_distance = math::CrossProduct({rotate_cs_x, rotate_cs_y}, 
                                                      {rotate_ce_x, rotate_ce_y},
                                                      {box_x, -box_y}) <= 0.0
                                 ? Vertex2SegmentDistance( box_x, -box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght())

                                 : math::CrossProduct({rotate_cs_x, rotate_cs_y}, 
                                                      {rotate_ce_x, rotate_ce_y},
                                                      {-box_x, box_y}) <= 0.0
                                 ? 0.0
                                 : Vertex2SegmentDistance(-box_x, box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;

                default:
                    min_distance = 0.0;
                    break;
            }
        }
        else
        {
            switch (3 * binary_ce_x + binary_ce_y)        
            {
                // (1, 1)
                case 4:
                    min_distance = rotate_cs_x < rotate_ce_x 
                                 ? rotate_cs_x - box_x 
                                 : Vertex2SegmentDistance( box_x, box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;

                // (1, 0)
                case 3:
                    min_distance = std::min(rotate_cs_x, rotate_ce_x) - box_x;
                    break;

                // (0, 1) or (-1, 1)
                case  1:
                case -2:
                    min_distance = math::CrossProduct({rotate_cs_x, rotate_cs_y}, 
                                                      {rotate_ce_x, rotate_ce_y},
                                                      {box_x, box_y}) <= 0.0
                                 ? 0.0
                                 : Vertex2SegmentDistance( box_x, box_y,
                                                           rotate_cs_x, rotate_cs_y,
                                                           rotate_ce_x, rotate_ce_y,
                                                           line_segment.getLenght());
                    break;

                // (-1, 0)
                case -3:
                    min_distance = 0.0;
                    break;

                default:
                    min_distance = 0.0;
                    break;
            }
        }

        return min_distance;
    }

    bool Box2d::HasOverlap(const LineSegment2d &line_segment) const
    {
        // the line-segment whether is a point
        if (line_segment.getLenght() <= kMathEpsilon)
        {
            return IsPointIn(line_segment.getStart());
        }

        // judgment the line-segment whether is outside the box
        if (std::fmax(line_segment.getStart().getX(), line_segment.getEnd().getX()) < getMin_x() ||
            std::fmin(line_segment.getStart().getX(), line_segment.getEnd().getX()) > getMax_x() ||
            std::fmax(line_segment.getStart().getY(), line_segment.getEnd().getY()) < getMin_y() ||
            std::fmax(line_segment.getStart().getY(), line_segment.getEnd().getY()) > getMax_y())
        {
            return false;
        }
        return DistanceTo(line_segment) <= kMathEpsilon;
    }

    /****** box ******/
    double Box2d::DistanceTo(const Box2d &box) const
    {
        return box.getHalfLength(); 
    }

    bool Box2d::HasOverlap(const Box2d &box) const
    {
        if (box.getMax_x() < getMin_x() 
         || box.getMin_x() > getMax_x()
         || box.getMax_y() < getMin_y()
         || box.getMin_y() > getMax_y())
        {
            return false;
        }

        const double shift_x = box.getCenterX() - getCenterX();
        const double shift_y = box.getCenterY() - getCenterY();

        const double dx1 =  _cos_heading * _half_length;
        const double dy1 =  _sin_heading * _half_length;
        const double dx2 =  _sin_heading * _half_width;
        const double dy2 = -_cos_heading * _half_width;

        const double dx3 =  box.getCosHeading() * _half_length;
        const double dy3 =  box.getSinHeading() * _half_length;
        const double dx4 =  box.getSinHeading() * _half_width;
        const double dy4 = -box.getCosHeading() * _half_width;

        return std::abs(shift_x * _cos_heading + shift_y * _sin_heading)
            <= std::abs(dx3 * _cos_heading + dy3 * _sin_heading)
             + std::abs(dx4 * _cos_heading + dy4 * _sin_heading)
             + _half_length
            && std::abs(shift_x * _sin_heading - shift_y * _cos_heading)
            <= std::abs(dx3 * _sin_heading - dy3 * _cos_heading)
             + std::abs(dx4 * _sin_heading - dy4 * _cos_heading)
             + _half_width
            && std::abs(shift_x * box.getCosHeading() + shift_y * box.getSinHeading())
            <= std::abs(dx1 * box.getCosHeading() + dy1 * box.getSinHeading())
             + std::abs(dx2 * box.getCosHeading() + dy2 * box.getSinHeading())
             + box.getHalfLength()
            && std::abs(shift_x * box.getSinHeading() - shift_y * box.getCosHeading())
            <= std::abs(dx1 * box.getSinHeading() - dy1 * box.getCosHeading())
             + std::abs(dx2 * box.getSinHeading() - dy2 * box.getCosHeading())
             + box.getHalfWidth();
    }
}
