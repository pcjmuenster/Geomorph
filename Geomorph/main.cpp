#include <iostream>

#include "map.h"
#include "noise.h"

int main()
{
    MapF noise = makeNoise(100, 100, .5f);

    std::cout << noise[0][0] << "\n";
    return 0;
}
