/*
 * math_utils.cpp
 * Copyright (C) 2020 guohua zhu <zgh_email@163.com>
 * 
 * Distributed under terms of the MIT license.
 */

#include "../include/math_utils.h"

namespace math
{


double CrossProduct(const Vector2d &start_point,
                    const Vector2d &end_point1,
                    const Vector2d &end_point2)
{
    return (end_point1 - start_point).CrossProduct(end_point2 - start_point); 
}

}
