#ifndef EROSION_H
#define EROSION_H

#include "map_fwd.h"

void erodeFluvial(DMap& terrain, int samples);

void erodeThermal(DMap& terrain, int samples);

#endif // EROSION_H
