#include <chrono>
#include <fstream>
#include <iostream>

#include "geomorph/map.h"
#include "geomorph/noise.h"
#include "geomorph/water.h"

#include "verify.h"

void output_image(const std::string& filename, const DMap& map)
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
    auto start = std::chrono::system_clock::now();

    NoiseParams params;
    params.seed = 4711;
    params.horizontal_wrapping = true;
    DMap terrain1 = make_noise(256, 256, params);
    DMap terrain{terrain1.size()};
    for (std::size_t x = 0; x < terrain.width(); ++x)
        for (std::size_t y = 0; y < terrain.height(); ++y) {
            terrain[x][y] = terrain1[(x + terrain.width() / 2) % terrain.width()][y];
        }

    DMap precipitation = DMap(terrain.width(), terrain.height(), 0.1);
    DMap stationary_water = make_water(terrain, precipitation);

    auto end = std::chrono::system_clock::now();
    using Unit = std::chrono::milliseconds;
    auto duration = std::chrono::duration_cast<Unit>(end - start).count();
    std::clog << duration << " ms\n";

    verify(terrain, precipitation, stationary_water);

    output_image("terrain.pgm", terrain);

    return 0;
}
