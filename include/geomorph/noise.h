#ifndef NOISE_H
#define NOISE_H

#include <cstddef>

#include "map_fwd.h"

struct NoiseParams {
    /**
     * @brief seed will be replaced by the current time if equal to zero
     */
    unsigned int seed = 0;
    /**
     * @brief roughness determines how smooth (smaller values) or rough (larger) the
     *                  resulting noise is; values should be chosen between 0.5 and 1.5
     */
    float roughness = .5f;
};

/**
 * @brief constructs a two-dimensional array of floats
 * @param width
 * @param height
 * @param roughness determines how smooth (smaller values) or rough (larger) the
 *                  resulting noise is; values should be chosen between 0.5 and 1.5
 */
DMap makeNoise(std::size_t width, std::size_t height, const NoiseParams& = NoiseParams{});

DMap perturbed(const DMap& map);

#endif // NOISE_H
