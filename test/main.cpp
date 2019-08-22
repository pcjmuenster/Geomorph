#include <chrono>
#include <iostream>

#include "geomorph/map.h"
#include "geomorph/noise.h"
#include "geomorph/water.h"

int main()
{
    auto start = std::chrono::system_clock::now();

    NoiseParams params;
    params.seed = 4711;
    DMap terrain = makeNoise(100, 100, params);
    DMap precipitation = DMap(terrain.width(), terrain.height(), 0.1);
    auto [stationaryWater, flowingWater] = addWater(terrain, precipitation);

    auto end = std::chrono::system_clock::now();
    using Unit = std::chrono::milliseconds;
    auto duration = std::chrono::duration_cast<Unit>(end - start).count();
    std::cout << duration << " ms\n";

    verify(terrain, precipitation, stationaryWater);

    return 0;
}
