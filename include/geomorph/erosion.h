#ifndef EROSION_H
#define EROSION_H

#include "map_fwd.h"

void erode_fluvial(DMap& terrain, int samples);

void erode_thermal(DMap& terrain, int samples);

#endif // EROSION_H
