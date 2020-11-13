/*
 * line_segment2d.cpp
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 * 
 * Distributed under terms of the MIT license.
 */

#include "../include/line_segment2d.h"

namespace math
{
    LineSegment2d::LineSegment2d()
    {
        _unit_direction = Vector2d(1, 0);
    }

    LineSegment2d::LineSegment2d(const Vector2d &start, const Vector2d &end)
        : _start(start), _end(end)
    {
        const double dx = _end.getX() - _start.getX();
        const double dy = _end.getY() - _start.getY();
        _length = hypot(dx, dy);
        _unit_direction = _length <= 1.0e-6 ? Vector2d(0, 0) : Vector2d(dx / _length, dy / _length);
        _heading = _unit_direction.Angle();
    }

    Vector2d LineSegment2d::rotate(const double angle)
    {
        Vector2d diff_vec = _end - _start;
        diff_vec.rotate(angle);
        return _start + diff_vec;
    }

    double LineSegment2d::getLength() const { return _length; }

    double LineSegment2d::getLengthSqr() const { return _length * _length; }

    double LineSegment2d::DistanceTo(const Vector2d &point) const
    {
        if (_length < kMathEpsilon)
        {
            return point.DistanceTo(_start);
        }

        // calculate the vector from start point to point
        const double sp_x = point.getX() - _start.getX();
        const double sp_y = point.getY() - _start.getY();

        // vector dot project 
        const double proj = sp_x * _unit_direction.getX() + sp_y * _unit_direction.getY();

        if (proj <= 0.0)
        {
            return hypot(sp_x, sp_y);
        }

        if (proj >= _length)
        {
            return point.DistanceTo(_end);
        }

        // the sp vctor cross product to the unit vector of lline segment
        return std::abs(sp_x * _unit_direction.getY() - sp_y * _unit_direction.getX());
    }
}

