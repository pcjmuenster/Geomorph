#include "geomorph/noise.h"

#include <algorithm>
#include <chrono>
#include <random>

#include "geomorph/map.h"
#include "geomorph/math_utils.h"

void normalize(DMap& noise)
{
    auto minmax = std::minmax_element(noise.begin(), noise.end());       
    auto min = *minmax.first;
    auto factor = 1 / (*minmax.second - min);
    for (auto& value : noise)
        value = (value - min) * factor;
}

DMap make_noise(std::size_t width, std::size_t height, const NoiseParams& params)
{
    auto seed = params.seed;
    if (seed == 0) {
        auto time = std::chrono::high_resolution_clock::now();
        seed = unsigned(time.time_since_epoch().count());
    }
    std::default_random_engine eng{seed};

    auto make_device = [&params](std::size_t level) {
        static constexpr std::size_t max_amplitude_level = 7;
        auto exp = max_amplitude_level - std::min(level, max_amplitude_level);
        double range = std::pow(params.roughness, double(exp));
        return std::uniform_real_distribution<double>{-range, range};
    };

    auto size = std::max(width, height);
    std::size_t level_of_detail = std::size_t(std::ceil(std::log2(size - 1)));
    size = (1 << level_of_detail) + 1;
    DMap noise(size, size);
    /* initialize corners */ {
        auto dev = make_device(level_of_detail);
        noise[    0   ][    0   ] = dev(eng);
        noise[    0   ][size - 1] = dev(eng);
        noise[size - 1][    0   ] = dev(eng);
        noise[size - 1][size - 1] = dev(eng);
    }

    for (auto level = level_of_detail; level > 0; --level) {
        auto dev = make_device(level);

        std::size_t stride = 1 << level;
        std::size_t half = stride / 2;
        for (std::size_t x = 0; x < size - stride; x += stride) {
            for (std::size_t y = 0; y < size - stride; y += stride) {
                double lb = noise[x         ][y         ];
                double lt = noise[x         ][y + stride];
                double rb = noise[x + stride][y         ];
                double rt = noise[x + stride][y + stride];

                double mm = (lb + lt + rb + rt) / 4 + dev(eng);
                noise[x + half][y + half] = mm;

                double lm = lb + mm + lt;
                if (x > 0)
                    lm = (lm + noise[x - half][y + half]) / 4;
                else
                    lm = lm / 3;
                noise[x][y + half] = lm + dev(eng);

                double mb = lb + rb + mm;
                if (y > 0)
                    mb = (mb + noise[x + half][y - half]) / 4;
                else
                    mb = mb / 3;
                noise[x + half][y] = mb + dev(eng);

                // last row
                if (y + stride == size - 1) {
                    double mt = (lt + rt + mm)/ 3;
                    noise[x + half][size - 1] = mt + dev(eng);
                }
                // last column
                if (x + stride == size - 1) {
                    double rm = (rb + rt + mm) / 3;
                    noise[size - 1][y + half] = rm + dev(eng);
                }
            }
        }
    }

    noise.resize(width, height);

    normalize(noise);

    return noise;
}

DMap perturbed(const DMap& map)
{
    auto width = map.width();
    auto height = map.height();

    DMap result(width, height);

    auto x_noise = make_noise(width, height);
    auto y_noise = make_noise(width, height);

    double magnitude = 0.25 * width;

    for (std::size_t x = 0u; x < width; ++x)
        for (std::size_t y = 0u; y < height; ++y) {
            double x_ = x + magnitude * (x_noise[x][y] - .5);
            double y_ = y + magnitude * (y_noise[x][y] - .5);
            x_ = clamp(0, x_, width - 1);
            y_ = clamp(0, y_, height - 1);
            result[x][y] = map[std::size_t(x_)][std::size_t(y_)];
        }
    return result;
}
