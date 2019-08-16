#include <iostream>
#include <limits>

#include "geomorph/map.h"
#include "geomorph/noise.h"
#include "geomorph/water.h"

int main()
{
    NoiseParams params;
    params.seed = 4711;
    DMap terrain = makeNoise(100, 100, params);
    DMap precipitation = DMap(terrain.width(), terrain.height(), 0.1);
    DMap water = addWater(terrain, precipitation);

    verify(terrain, precipitation, water);

    return 0;
}
