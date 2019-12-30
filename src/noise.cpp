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

/*
 * The following scheme is used to name variables:
 * (x, y + stride)       (x + stride, y + stride)
 *      lt --------- mt ---------- rt
 *      |             :             |
 *      |             :             |
 *      |             :             |
 *      lm ......... mm .......... rm
 *      |             :             |
 *      |             :             |
 *      |             :             |
 *      lb --------- mb ---------- rb
 *   (x, y)                   (x + stride, y)
 */

template <class RandomDevice>
void compute_centers(const NoiseParams& params, DMap& noise,
                     std::size_t stride, RandomDevice& dev)
{
    auto half = stride / 2;
    for (std::size_t x = 0; x < noise.width() - stride; x += stride)
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            double lb = noise[x         ][y         ];
            double lt = noise[x         ][y + stride];
            double rb = noise[x + stride][y         ];
            double rt = noise[x + stride][y + stride];

            double mm = (lb + lt + rb + rt) / 4 + dev();
            noise[x + half][y + half] = mm;
        }

    if (params.horizontal_wrapping) {
        std::size_t x = noise.width() - stride;
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            double lb = noise[x][y         ];
            double lt = noise[x][y + stride];
            double rb = noise[0][y         ];
            double rt = noise[0][y + stride];

            double mm = (lb + lt + rb + rt) / 4 + dev();
            noise[x + half][y + half] = mm;
        }
    }

    if (params.vertical_wrapping) {
        std::size_t y = noise.height() - stride;
        for (std::size_t x = 0; x < noise.width() - stride; x += stride) {
            double lb = noise[x         ][y];
            double lt = noise[x         ][0];
            double rb = noise[x + stride][y];
            double rt = noise[x + stride][0];

            double mm = (lb + lt + rb + rt) / 4 + dev();
            noise[x + half][y + half] = mm;
        }
    }

    if (params.horizontal_wrapping && params.vertical_wrapping) {
        std::size_t x = noise.width() - stride;
        std::size_t y = noise.height() - stride;

        double lb = noise[x][y];
        double lt = noise[x][0];
        double rb = noise[0][y];
        double rt = noise[0][0];

        double mm = (lb + lt + rb + rt) / 4 + dev();
        noise[x + half][y + half] = mm;
    }

}

template <class RandomDevice>
void compute_vertical_edge_centers(const NoiseParams& params, DMap& noise,
                                   std::size_t stride, RandomDevice& dev)
{
    std::size_t half = stride / 2;
    // interior edges
    for (std::size_t x = stride; x < noise.width() - half; x += stride)
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            double lb  = noise[x       ][y         ];
            double lt  = noise[x       ][y + stride];
            double mm  = noise[x + half][y + half];
            double lmm = noise[x - half][y + half];

            double lm = (lb + mm + lt + lmm) / 4;
            noise[x][y + half] = lm + dev();
        }

    if (params.vertical_wrapping) {
        std::size_t y = noise.height() - stride;
        for (std::size_t x = stride; x < noise.width() - half; x += stride) {
            double lb  = noise[x       ][y       ];
            double lt  = noise[x       ][0       ];
            double mm  = noise[x + half][y + half];
            double lmm = noise[x - half][y + half];

            double lm = (lb + mm + lt + lmm) / 4;
            noise[x][y + half] = lm + dev();
        }
    }

    if (params.horizontal_wrapping) {
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            double lb  = noise[0][y         ];
            double lt  = noise[0][y + stride];
            double mm  = noise[half][y + half];
            double lmm = noise[noise.width() - half][y + half];

            double lm = (lb + mm + lt + lmm) / 4;
            noise[0][y + half] = lm + dev();
        }
        // topmost left edge
        if (params.vertical_wrapping) {
            double lb  = noise[0][noise.height() - stride];
            double lt  = noise[0][0];
            double mm  = noise[half][noise.height() - half];
            double lmm = noise[noise.width() - half][noise.height() - half];

            double lm = (lb + mm + lt + lmm) / 4;
            noise[0][noise.height() - half] = lm + dev();
        }
    }
    else {
        // left edges
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            double lb  = noise[0][y         ];
            double lt  = noise[0][y + stride];
            double mm  = noise[half][y + half];

            double lm = (lb + mm + lt) / 3;
            noise[0][y + half] = lm + dev();
        }
        // right edges
        for (std::size_t y = 0; y < noise.height() - stride; y += stride) {
            auto x = noise.width() - 1 - stride;
            double rb  = noise[x + stride][y         ];
            double rt  = noise[x + stride][y + stride];
            double mm  = noise[x + half  ][y + half];

            double rm = (rb + mm + rt) / 3;
            noise[x + stride][y + half] = rm + dev();
        }

        if (params.vertical_wrapping) {
            auto y = noise.height() - stride;
            // topmost left edge
            {
                double lb  = noise[0][y];
                double lt  = noise[0][0];
                double mm  = noise[half][y + half];

                double lm = (lb + mm + lt) / 3;
                noise[0][y + half] = lm + dev();
            }
            // topmost right edge
            {
                auto x = noise.width() - 1 - stride;
                double rb  = noise[x + stride][y];
                double rt  = noise[x + stride][0];
                double mm  = noise[x + half  ][y + half];

                double rm = (rb + mm + rt) / 3;
                noise[x + stride][y + half] = rm + dev();
            }
        }
    }

}

template <class RandomDevice>
void compute_horizontal_edge_centers(const NoiseParams& params, DMap& noise,
                                     std::size_t stride, RandomDevice& dev)
{
    std::size_t half = stride / 2;
    for (std::size_t x = 0; x < noise.width() - stride; x += stride)
        for (std::size_t y = stride; y < noise.height() - half; y += stride) {
            double lb  = noise[x         ][y       ];
            double rb  = noise[x + stride][y       ];
            double mm  = noise[x + half  ][y + half];
            double bmm = noise[x + half  ][y - half];

            double mb = (lb + rb + mm + bmm) / 4;
            noise[x + half][y] = mb + dev();
        }

    if (params.horizontal_wrapping) {
        std::size_t x = noise.width() - stride;
        for (std::size_t y = stride; y < noise.height() - half; y += stride) {
            double lb  = noise[x       ][y       ];
            double rb  = noise[0       ][y       ];
            double mm  = noise[x + half][y + half];
            double bmm = noise[x + half][y - half];

            double mb = (lb + rb + mm + bmm) / 4;
            noise[x + half][y] = mb + dev();
        }
    }

    if (params.vertical_wrapping) {
        for (std::size_t x = 0; x < noise.width() - stride; x += stride) {
            double lb  = noise[x         ][0];
            double rb  = noise[x + stride][0];
            double mm  = noise[x + half][half];
            double bmm = noise[x + half][noise.height() - half];

            double mb = (lb + mm + rb + bmm) / 4;
            noise[x + half][0] = mb + dev();
        }
        // rightmost bottom edge
        if (params.horizontal_wrapping) {
            double lb  = noise[noise.width() - stride][0];
            double rb  = noise[0][0];
            double mm  = noise[noise.width() - half][half];
            double bmm = noise[noise.width() - half][noise.height() - half];

            double mb = (lb + mm + rb + bmm) / 4;
            noise[0][noise.height() - half] = mb + dev();
        }
    }
    else {
        // bottom edges
        for (std::size_t x = 0; x < noise.width() - stride; x += stride) {
            double lb = noise[x         ][0];
            double rb = noise[x + stride][0];
            double mm = noise[x + half][half];

            double mb = (lb + mm + rb) / 3;
            noise[x + half][0] = mb + dev();
        }
        // top edges
        for (std::size_t x = 0; x < noise.width() - stride; x += stride) {
            auto y = noise.height() - 1 - stride;
            double lt = noise[x][y + stride];
            double rt = noise[x + stride][y + stride];
            double mm = noise[x + half  ][y + half];

            double mt = (lt + mm + rt) / 3;
            noise[x + half][y + stride] = mt + dev();
        }

        if (params.horizontal_wrapping) {
            auto x = noise.width() - stride;
            // rightmost bottom edge
            {
                double lb = noise[x][0];
                double rb = noise[0][0];
                double mm = noise[x + half][half];

                double mb = (lb + rb + mm) / 3;
                noise[x + half][0] = mb + dev();
            }
            // rightmost top edge
            {
                auto y = noise.height() - 1 - stride;
                double lt = noise[x][y + stride];
                double rt = noise[0][y + stride];
                double mm = noise[x + half][y + half];

                double mt = (lt + mm + rt) / 3;
                noise[x + half][y + stride] = mt + dev();
            }
        }
    }
}

DMap make_noise(std::size_t width, std::size_t height, const NoiseParams& params)
{
    auto seed = params.seed;
    if (seed == 0) {
        auto time = std::chrono::high_resolution_clock::now();
        seed = unsigned(time.time_since_epoch().count());
    }
    std::default_random_engine eng{seed};

    auto make_device = [&params, &eng](std::size_t level) {
        static constexpr std::size_t max_amplitude_level = 7;
        auto exp = max_amplitude_level - std::min(level, max_amplitude_level);
        double range = std::pow(params.roughness, double(exp));
        std::uniform_real_distribution<double> dist{-range, range};
        return [dist, &eng] () mutable { return dist(eng); };
    };

    auto size = std::max(width, height);
    auto level_of_detail = std::size_t(std::ceil(std::log2(size)));
    auto w = std::size_t(1 << level_of_detail);
    if (not params.horizontal_wrapping)
        ++w;
    auto h = std::size_t(1 << level_of_detail);
    if (not params.vertical_wrapping)
        ++h;
    DMap noise(w, h);
    /* initialize corners */ {
        auto dev = make_device(level_of_detail);
            noise[  0  ][  0  ] = dev();
        if (not params.horizontal_wrapping)
            noise[w - 1][  0  ] = dev();
        if (not params.vertical_wrapping)
            noise[  0  ][h - 1] = dev();
        if (not params.horizontal_wrapping && not params.vertical_wrapping)
            noise[w - 1][h - 1] = dev();
    }

    for (auto level = level_of_detail; level > 0; --level) {
        auto dev = make_device(level);
        std::size_t stride = 1 << level;        
        compute_centers(params, noise, stride, dev);
        compute_vertical_edge_centers(params, noise, stride, dev);
        compute_horizontal_edge_centers(params, noise, stride, dev);
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
