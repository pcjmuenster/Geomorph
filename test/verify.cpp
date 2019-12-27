#include "verify.h"

#include <iostream>

#include "geomorph/map.h"
#include "geomorph/math_utils.h"

void verify(const DMap& terrain, const DMap& precipitation,
            const DMap& water)
{
    for (std::size_t x = 0; x < terrain.width(); ++x)
        for (std::size_t y = 1; y < terrain.height(); ++y) {
            double h1 = terrain[x][y - 1];
            double h2 = terrain[x][  y  ];
            double w1 = water[x][y - 1];
            double w2 = water[x][  y  ];

            if (h1 > h2 + w2) {
                if(not is_zero(w1))
                    std::cerr << "water not zero at (" << x - 1 << ", " << y << ")\n";
            }
            else if (h2 > h1 + w1) {
                if(not is_zero(w2))
                    std::cerr << "water not zero at (" << x << ", " << y << ")\n";
            }
            else {
                if(not is_equal(h1 + w1, h2 + w2))
                    std::cerr << "water not even at (" << x << ", " << y << ")\n";
            }
        }
    for (std::size_t x = 1; x < terrain.width(); ++x)
        for (std::size_t y = 0; y < terrain.height(); ++y) {
            double h1 = terrain[x - 1][y];
            double h2 = terrain[  x  ][y];
            double w1 = water[x - 1][y];
            double w2 = water[  x  ][y];

            if (h1 > h2 + w2) {
                if(not is_zero(w1))
                    std::cerr << "water not zero at (" << x << ", " << y - 1 << ")\n";
            }
            else if (h2 > h1 + w1) {
                if(not is_zero(w2))
                    std::cerr << "water not zero at (" << x << ", " << y << ")\n";
            }
            else {
                if(not is_equal(h1 + w1, h2 + w2))
                    std::cerr << "water not even at (" << x << ", " << y << ")\n";
            }
        }

    double total_humidity = 0;
    for (double h : precipitation)
        total_humidity += h;

    double total_water = 0;
    for (double w : water)
        total_water += w;

    if (not is_equal(total_humidity, total_water))
        std::cout << "water lost: " << total_humidity - total_water << '\n';
}
