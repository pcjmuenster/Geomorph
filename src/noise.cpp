#include "geomorph/noise.h"

#include <algorithm>
#include <chrono>
#include <random>

#include "geomorph/map.h"
#include "geomorph/math_utils.h"

void normalize(MapF& noise)
{
    auto minmax = std::minmax_element(noise.begin(), noise.end());       
    float min = *minmax.first;
    float factor = 1 / (*minmax.second - min);
    for (float& value : noise)
        value = (value - min) * factor;
}

MapF makeNoise(std::size_t width, std::size_t height, float roughness)
{
    auto time = std::chrono::high_resolution_clock::now();
    auto seed = unsigned(time.time_since_epoch().count());    
    std::default_random_engine eng{seed};

    auto makeDevice = [roughness](std::size_t level) {
        constexpr std::size_t max_amplitude_level = 7;
        auto exp = max_amplitude_level - std::min(level, max_amplitude_level);
        float range = std::pow(roughness, float(exp));
        return std::uniform_real_distribution<float>{-range, range};
    };

    auto size = std::max(width, height);
    std::size_t levelOfDetail = std::size_t(std::ceil(std::log2(size - 1)));
    size = (1 << levelOfDetail) + 1;
    MapF noise(size, size);
    /* initialize corners */ {
        auto dev = makeDevice(levelOfDetail);
        noise[    0   ][    0   ] = dev(eng);
        noise[    0   ][size - 1] = dev(eng);
        noise[size - 1][    0   ] = dev(eng);
        noise[size - 1][size - 1] = dev(eng);
    }

    for (auto level = levelOfDetail; level > 0; --level) {
        auto dev = makeDevice(level);

        std::size_t stride = 1 << level;
        std::size_t half = stride / 2;
        for (std::size_t x = 0; x < size - stride; x += stride) {
            for (std::size_t y = 0; y < size - stride; y += stride) {
                float lb = noise[x         ][y         ];
                float lt = noise[x         ][y + stride];
                float rb = noise[x + stride][y         ];
                float rt = noise[x + stride][y + stride];

                float mm = (lb + lt + rb + rt) / 4 + dev(eng);
                noise[x + half][y + half] = mm;

                float lm = lb + mm + lt;
                if (x > 0)
                    lm = (lm + noise[x - half][y + half]) / 4;
                else
                    lm = lm / 3;
                noise[x][y + half] = lm + dev(eng);

                float mb = lb + rb + mm;
                if (y > 0)
                    mb = (mb + noise[x + half][y - half]) / 4;
                else
                    mb = mb / 3;
                noise[x + half][y] = mb + dev(eng);

                // last row
                if (y + stride == size - 1) {
                    float mt = (lt + rt + mm)/ 3;
                    noise[x + half][size - 1] = mt + dev(eng);
                }
                // last column
                if (x + stride == size - 1) {
                    float rm = (rb + rt + mm) / 3;
                    noise[size - 1][y + half] = rm + dev(eng);
                }
            }
        }
    }

    noise.resize(width, height);

    normalize(noise);

    return noise;
}

MapF perturbed(const MapF& map)
{
    auto width = map.width();
    auto height = map.height();

    MapF result(width, height);
    auto xNoise = makeNoise(width, height, 0.5f);
    auto yNoise = makeNoise(width, height, 0.5f);

    float magnitude = 0.25f * width;

    for (auto x = 0u; x < width; ++x)
        for (auto y = 0u; y < height; ++y) {
            float x_ = x + magnitude * (xNoise[x][y] - .5f);
            float y_ = y + magnitude * (yNoise[x][y] - .5f);
            x_ = clamp(0, x_, width - 1);
            y_ = clamp(0, y_, height - 1);
            result[x][y] = map[std::size_t(x_)][std::size_t(y_)];
        }
    return result;
}
