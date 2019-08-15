#ifndef VEC2F_H
#define VEC2F_H

/* A class that represents a two-diamensional vector, currently no additional
 * functionality is needed -- which this kind of useless.
 */

class Vec2F
{
public:
    Vec2F() = default;
    Vec2F(float x, float y);

    float x, y;
};

#endif // VEC2F_H
