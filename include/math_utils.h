/*
 * math_utils.h
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Dense>

#include "vector_2d.h"


namespace math
{

/**
 * @brief Cross product between 2-d vectors from the common start point,
 *        and end at two other points.
 * @param start_point: The common start point of two vectors in 2-D.
 * @param end_point1: The end point of the first vector.
 * @param end_point2: The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProduct(const Vector2d &start_point,
                    const Vector2d &end_point1,
                    const Vector2d &end_point2);

}

#endif /* !MATH_UTILS_H */
