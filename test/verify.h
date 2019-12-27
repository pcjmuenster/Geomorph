#ifndef VERIFY_H
#define VERIFY_H

#include "geomorph/map_fwd.h"

/**
 * @brief verify check the result of add_water (with depthThreshold = 0)
 *               for inconsistencies in stationary water
 * @param terrain
 * @param precipitation
 * @param water         the first element of the return value of add_water
 */
void verify(const DMap& terrain, const DMap& precipitation, const DMap& water);

#endif // VERIFY_H
