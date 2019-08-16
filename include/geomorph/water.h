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
void diffuseWater(const DMap& terrain, DMap& water, int iterations);

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
 * @param depthThreshold the minimum depth for water basins;
 *                       more shallow pits will be removed
 * @return
 */
DMap addWater(const DMap& terrain, const DMap& precipitation, double depthThreshold = 0);

/**
 * @brief verify check the result of addWater (with depthThreshold = 0)
 *               for inconsistencies
 * @param terrain
 * @param precipitation
 * @param water
 */
void verify(const DMap& terrain, const DMap& precipitation, const DMap& water);

void advectHumidity(const D2Map& wind, DMap& humidity, int iterations);

#endif // WATER_H
