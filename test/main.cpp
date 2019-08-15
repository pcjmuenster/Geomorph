#include <iostream>

#include "geomorph/map.h"
#include "geomorph/noise.h"
#include "geomorph/water.h"

int main()
{
    MapF terrain = makeNoise(100, 100, .5f);
    MapF precipitation = MapF(terrain.width(), terrain.height(), 0.1f);
    MapF water = addWater(terrain, precipitation);

    return 0;
}
