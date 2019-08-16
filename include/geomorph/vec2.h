#ifndef VEC2_H
#define VEC2_H

/* A class that represents a two-diamensional vector, currently no additional
 * functionality is needed -- which this kind of useless.
 */

class Vec2
{
public:
    Vec2() = default;
    Vec2(double x, double y);

    double x, y;
};

#endif // VEC2_H
