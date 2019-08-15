#include "geomorph/erosion.h"

#include <cmath>
#include <random>

#include "geomorph/map.h"

void erodeFluvial(MapF& terrain, int samples)
{
    const auto mapWidth = terrain.width();
    const auto mapHeight = terrain.height();

    std::default_random_engine eng{};
    std::uniform_int_distribution<size_t> xDev{0, mapWidth - 1};
    std::uniform_int_distribution<size_t> yDev{0, mapHeight - 1};

    for (int i = 0; i < samples; ++i) {
        auto x = xDev(eng);
        auto y = yDev(eng);

        float h = terrain[x][y];
        float sediment = 0;
        float volume = 0.3f;
        float speed = 0;
        while (true) {
            auto nextX = x;
            auto nextY = y;
            float currMin = std::numeric_limits<float>::max();
            if (x > 0) {
                float neighbor = terrain[x - 1][y];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x - 1;
                    nextY = y;
                }
            }
            if (x + 1 < mapWidth) {
                float neighbor = terrain[x + 1][y];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x + 1;
                    nextY = y;
                }
            }
            if (y > 0) {
                float neighbor = terrain[x][y - 1];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x;
                    nextY = y - 1;
                }
            }
            if (y + 1 < mapHeight) {
                float neighbor = terrain[x][y + 1];
                if (neighbor < currMin) {
                    currMin = neighbor;
                    nextX = x;
                    nextY = y + 1;
                }
            }

            if (currMin < h) {
                h = currMin;
                float diff = terrain[x][y] - h;
                speed = 0.3f * speed + 0.7f * diff;
                float capacity = volume * speed; // TODO;
                if (sediment > capacity) {
                    // dispose
                    float delta = 0.3f * (sediment - capacity);
                    terrain[x][y] += delta;
                    sediment -= delta;
                }
                else {
                    // erode
                    float delta = std::min(0.5f * (capacity - sediment), diff - 0.001f);
                    terrain[x][y] -= 0.5f * delta;
                    bool horizontalFlow = x - nextX != 0;
                    if (horizontalFlow) {
                        if (y > 0)
                            terrain[x][y - 1] -= 0.25f * delta;
                        if (y + 1 < mapHeight)
                            terrain[x][y + 1] -= 0.25f * delta;
                    }
                    else {
                        if (x > 0)
                            terrain[x - 1][y] -= 0.25f * delta;
                        if (x + 1 < mapWidth)
                            terrain[x + 1][y] -= 0.25f * delta;
                    }


                    sediment += delta;
                }

                volume *= 0.975f;

                x = nextX;
                y = nextY;
            }
            else {
                // end flow
                if (currMin - h < sediment)
                    terrain[x][y] = currMin + 0.001f;
                else
                    terrain[x][y] += sediment;

                break;
            }
        }
    }
}

void erodeThermal(MapF& terrain, int samples)
{
    const auto mapWidth = terrain.width();
    const auto mapHeight = terrain.height();
    const float talus = 2.f / std::min(mapWidth, mapHeight);

    std::default_random_engine eng{};
    std::uniform_int_distribution<size_t> xDev{1, mapWidth - 2};
    std::uniform_int_distribution<size_t> yDev{1, mapHeight - 2};

    for (int i = 0; i < samples; ++i) {
        auto x = xDev(eng);
        auto y = yDev(eng);
        float h = terrain[x][y];
        float maxSlide = 0;
        float slides[3][3] = {};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                if (i == 1 && j == 1) // ignore center
                    continue;
                float diff = h - terrain[x + i - 1][y + j - 1];
                float dist = std::hypot(i - 1.f, j - 1.f);
                if (diff > talus * dist) {
                    slides[i][j] = (diff - talus) / 2;
                    if (diff > maxSlide)
                        maxSlide = diff;
                }
            }

        if (maxSlide > 0.01f) {
            maxSlide *= 0.5f;
            float totalSlide = 0;
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
