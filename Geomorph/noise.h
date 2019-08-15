#ifndef NOISE_H
#define NOISE_H

#include <cstddef>

#include "map_fwd.h"

/**
 * @brief The NoiseParams struct bundles various parameters for the makeNoise function
 */
struct NoiseParams {
    /**
     * @brief roughness determines how smooth (smaller values) or rough (larger) the
     *                  resulting noise is; values should be chosen between 0.5 and 1.5
     */
    float roughness = 0.5f;
    /**
     * @brief seed will use the current time instead if equal to -1u
     */
    unsigned int seed = -1u;
};

/**
 * @brief constructs a two-dimensional array of floats
 * @param width
 * @param height
 * @param params
 */
MapF makeNoise(std::size_t width, std::size_t height,
               const NoiseParams& params = NoiseParams{});

MapF perturbed(const MapF& map);

#endif // NOISE_H
