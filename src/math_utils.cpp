#include "geomorph/math_utils.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

bool is_zero(double f)
{    
    return std::abs(f) < 1e-9;
}

bool is_equal(double lhs, double rhs)
{
    return is_zero(rhs) ? is_zero(lhs) : is_zero(lhs / rhs - 1);
}

double clamp(double min, double value, double max)
{
    return std::min(std::max(min, value), max);
}
