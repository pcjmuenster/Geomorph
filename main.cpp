#include <chrono>
#include <fstream>
#include <iostream>

#include "geomorph/map.h"
#include "geomorph/noise.h"
#include "geomorph/water.h"

void outputImage(const std::string& filename, const MapF& map)
{
    std::ofstream os{filename, std::ios_base::binary};
    if (not os.is_open())
        std::cerr << "could not open file " << filename;

    os << "P5 " << std::to_string(map.width()) << ' '
       << std::to_string(map.height()) << " 255 ";

    for (std::size_t y = 0; y < map.height(); ++y)
        for (std::size_t x = 0; x < map.width(); ++x)
            os << char(255 * map[x][y]);

    os.close();
}

int main()
{
    for (unsigned s = 0; s < 100; ++s) {
        auto start = std::chrono::high_resolution_clock::now();

        std::cout << s;
        NoiseParams params;
        params.roughness = .5f;
        params.seed = s;
        MapF terrain = makeNoise(250, 250, params);
        MapF precipitation = MapF(terrain.width(), terrain.height(), 0.1f);
        auto stationaryWater = addWater(terrain, precipitation);
        verify(terrain, precipitation, stationaryWater);

        auto end = std::chrono::high_resolution_clock::now();
        using Unit = std::chrono::milliseconds;
        auto duration = std::chrono::duration_cast<Unit>(end - start);
        std::cout << "time: " << duration.count() << " ms\n";
    }

//    outputImage("terrain.pgm", terrain);
//    outputImage("water.pgm", stationaryWater);

//    MapF total{terrain.size()};
//    for (std::size_t x = 0; x < terrain.width(); ++x)
//        for (std::size_t y = 0; y < terrain.height(); ++y)
//            total[x][y] = terrain[x][y] + stationaryWater[x][y];
//    outputImage("total.pgm", total);

    return 0;
}
