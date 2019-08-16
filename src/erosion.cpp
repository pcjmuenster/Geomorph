#include "geomorph/erosion.h"

#include <cmath>
#include <random>

#include "geomorph/map.h"

void erodeFluvial(DMap& terrain, int samples)
{
    const auto mapWidth = terrain.width();
    const auto mapHeight = terrain.height();

    std::default_random_engine eng{};
    std::uniform_int_distribution<size_t> xDev{0, mapWidth - 1};
    std::uniform_int_distribution<size_t> yDev{0, mapHeight - 1};

    for (int i = 0; i < samples; ++i) {
        auto x = xDev(eng);
        auto y = yDev(eng);

        double h = terrain[x][y];
        double sediment = 0;
        double volume = 0.3;
        double speed = 0;
        while (true) {
            auto nextX = x;
            auto nextY = y;
            double currMin = std::numeric_limits<double>::max();
            if (x > 0) {
                double neighbor = terrain[x - 1][y];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x - 1;
                    nextY = y;
                }
            }
            if (x + 1 < mapWidth) {
                double neighbor = terrain[x + 1][y];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x + 1;
                    nextY = y;
                }
            }
            if (y > 0) {
                double neighbor = terrain[x][y - 1];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x;
                    nextY = y - 1;
                }
            }
            if (y + 1 < mapHeight) {
                double neighbor = terrain[x][y + 1];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x;
                    nextY = y + 1;
                }
            }

            if (currMin < h) {
                h = currMin;
                double diff = terrain[x][y] - h;
                speed = 0.3 * speed + 0.7 * diff;
                double capacity = volume * speed; // TODO;
                if (sediment > capacity) {
                    // dispose
                    double delta = 0.3 * (sediment - capacity);
                    terrain[x][y] += delta;
                    sediment -= delta;
                }
                else {
                    // erode
                    double delta = std::min(0.5 * (capacity - sediment), diff - 0.001);
                    terrain[x][y] -= 0.5 * delta;
                    bool horizontalFlow = x - nextX != 0;
                    if (horizontalFlow) {
                        if (y > 0)
                            terrain[x][y - 1] -= 0.25 * delta;
                        if (y + 1 < mapHeight)
                            terrain[x][y + 1] -= 0.25 * delta;
                    }
                    else {
                        if (x > 0)
                            terrain[x - 1][y] -= 0.25 * delta;
                        if (x + 1 < mapWidth)
                            terrain[x + 1][y] -= 0.25 * delta;
                    }


                    sediment += delta;
                }

                volume *= 0.975;

                x = nextX;
                y = nextY;
            }
            else {
                // end flow
                if (currMin - h < sediment)
                    terrain[x][y] = currMin + 0.001;
                else
                    terrain[x][y] += sediment;

                break;
            }
        }
    }
}

void erodeThermal(DMap& terrain, int samples)
{
    const auto mapWidth = terrain.width();
    const auto mapHeight = terrain.height();
    const double talus = 2.0 / std::min(mapWidth, mapHeight);

    std::default_random_engine eng{};
    std::uniform_int_distribution<size_t> xDev{1, mapWidth - 2};
    std::uniform_int_distribution<size_t> yDev{1, mapHeight - 2};

    for (int i = 0; i < samples; ++i) {
        auto x = xDev(eng);
        auto y = yDev(eng);
        double h = terrain[x][y];
        double maxSlide = 0;
        double slides[3][3] = {};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                if (i == 1 && j == 1) // ignore center
                    continue;
                double diff = h - terrain[x + i - 1][y + j - 1];
                double dist = std::hypot(i - 1.0, j - 1.0);
                if (diff > talus * dist) {
                    slides[i][j] = (diff - talus) / 2;
                    if (diff > maxSlide)
                        maxSlide = diff;
                }
            }

        if (maxSlide > 0.01) {
            maxSlide *= 0.5;
            double totalSlide = 0;
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    totalSlide += slides[i][j];
            if (maxSlide < totalSlide) {
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        slides[i][j] *= maxSlide / totalSlide;
                totalSlide = maxSlide;
            }
            terrain[x][y] -= totalSlide;
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    terrain[x + i - 1][y + j - 1] += slides[i][j];
        }
    }
}
