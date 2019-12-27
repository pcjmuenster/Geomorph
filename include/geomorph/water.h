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
void diffuse_water(const DMap& terrain, DMap& water, int iterations);

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
 * @param depth_threshold the minimum depth for water basins;
 *                        more shallow pits will be removed
 * @return a floating-point Map of the same size as terrain that denotes the amount
 *         of stationary water
 */
DMap make_water(const DMap& terrain, const DMap& precipitation,
                double depth_threshold = 0);

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
 * @param depth_threshold the minimum depth for water basins;
 *                        more shallow pits will be removed
 * @return a pair of floating-point Maps of the same size as terrain where
 *         the first DMap denotes the amount of stationary water and
 *         the second denotes the amount of flowing water
 *         -- the two are mutually exclusive
 */
std::pair<DMap, DMap>
make_water_with_flow(const DMap& terrain, const DMap& precipitation,
                     double depth_threshold = 0);

void add_rivers(DMap& terrain, DMap& water);

void advect_humidity(const D2Map& wind, const DMap& humidity, int iterations);

#endif // WATER_H
