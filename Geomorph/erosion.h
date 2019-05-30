#ifndef EROSION_H
#define EROSION_H

#include "map_fwd.h"

void erodeFluvial(MapF& terrain, int samples);

void erodeThermal(MapF& terrain, int samples);

#endif // EROSION_H
