#ifndef WATER_H
#define WATER_H

#include <cstddef>
#include <utility>

#include "map_fwd.h"

/**
 * @brief simulates the water flow over the terrain such that the
 *        water level approximates an equilibrium after sufficiently many
 *        iterations
 * @param terrain
 * @param water
 * @param iterations
 */
void diffuseWater(const MapF& terrain, MapF& water, int iterations);

/**
 * @brief computes the equilibrium of the water distributed according to
 *        precipitation on the terrain such that there is no flow between adjacent
 *        cells necessary
 *
 * This function ignores possible evaporation, soil absorption or the inertia of
 * flowing water since no simulation of the flow process takes place; the equilibrium
 * is computed in a more direct manner which leads to an exact result
 * @param terrain
 * @param precipitation
 * @return
 */
MapF addWater(const MapF& terrain, const MapF& precipitation);

void advectHumidity(const Map2F& wind, MapF& humidity, int iterations);

#endif // WATER_H
