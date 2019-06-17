#include "math_utils.h"

#include <cmath>
#include <limits>

bool isZero(float f)
{
    return std::abs(f) < std::numeric_limits<float>::epsilon();
}

bool isEqual(float lhs, float rhs)
{
    return isZero(lhs - rhs);
}

float clamp(float min, float value, float max)
{
    return std::min(std::max(min, value), max);
}
