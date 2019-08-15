#ifndef NOISE_H
#define NOISE_H

#include <cstddef>

#include "map_fwd.h"

/**
 * @brief constructs a two-dimensional array of floats
 * @param width
 * @param height
 * @param roughness determines how smooth (smaller values) or rough (larger) the
 *                  resulting noise is; values should be chosen between 0.5 and 1.5
 */
MapF makeNoise(std::size_t width, std::size_t height, float roughness);

MapF perturbed(const MapF& map);

#endif // NOISE_H
