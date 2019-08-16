#include "geomorph/math_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>

bool isZero(double f)
{    
    return std::abs(f) < 1e-9;
}

bool isEqual(double lhs, double rhs)
{
    return isZero(lhs - rhs);
}

double clamp(double min, double value, double max)
{
    return std::min(std::max(min, value), max);
}
