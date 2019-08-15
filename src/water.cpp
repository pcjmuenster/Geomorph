#include "geomorph/water.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <tuple>

#include "geomorph/map.h"
#include "geomorph/math_utils.h"
#include "geomorph/vec2f.h"

void diffuseWater(const MapF& terrain, MapF& water, std::size_t x, std::size_t y)
{
    float totalWater = 0;
    totalWater += water[  x  ][  y  ];
    totalWater += water[  x  ][y + 1];
    totalWater += water[x + 1][  y  ];
    totalWater += water[x + 1][y + 1];
    if (totalWater < 0.001f)
        return;

    float heights[4] {
        terrain[  x  ][  y  ],
        terrain[  x  ][y + 1],
        terrain[x + 1][  y  ],
        terrain[x + 1][y + 1]
    };
    std::sort(heights, heights + 4);
    float level = heights[0];
    int tiles = 1;
    for (int i = 1; i < 4; ++i) {
        float neededWater = (heights[i] - level) * i;
        if (neededWater < totalWater) {
            totalWater -= neededWater;
            level = heights[i];
            ++tiles;
        }
        else break;
    }
    level += totalWater / tiles;
    auto riseWater = [&water, &terrain, level](std::size_t x, std::size_t y){
        water[x][y] = terrain[x][y] < level ? level - terrain[x][y] : 0;
    };
    riseWater(  x  ,   y  );
    riseWater(  x  , y + 1);
    riseWater(x + 1,   y  );
    riseWater(x + 1, y + 1);
}

void diffuseWater(const MapF& terrain, MapF& water, int iterations)
{
    MapF oldWater = water;

    auto width = terrain.width();
    auto height = terrain.height();
    for (int n = 0; n < iterations; ++n) {
        for (auto x = 0u; x < width - 1; ++x)
            for (auto y = 0u; y < height - 1; ++y) {
                diffuseWater(terrain, water, x, y);
            }
        for (auto y = height - 1; y > 0 ; --y)
            for (auto x = width - 1; x > 0; --x) {
                diffuseWater(terrain, water, x - 1, y - 1);
            }
    }

    float max = 0;
    for (auto x = 0u; x < width - 1; ++x)
        for (auto y = 0u; y < height - 1; ++y) {
            float diff = std::abs(water[x][y] - oldWater[x][y]);
            if (diff > max)
                max = diff;
        }
}

auto makeHeightComp(const MapF& map)
{
    return [&map](const Index& lhs, const Index& rhs) {
        return std::make_tuple(map[lhs], lhs) < std::make_tuple(map[rhs], rhs);
    };
}

std::vector<Index> sortedByHeight(const MapF& terrain)
{
    auto comp = makeHeightComp(terrain);

    std::vector<Index> indices;
    indices.reserve(terrain.width() * terrain.height());
    for (auto x = 0u; x < terrain.width(); ++x)
        for (auto y = 0u; y < terrain.height(); ++y)
            indices.push_back({x, y});
    std::sort(indices.begin(), indices.end(), comp);

    return indices;
}

Map<Index> computeSinks(const MapF& terrain, const std::vector<Index>& indices)
{
    Map<Index> sinks(terrain.width(), terrain.height());
    for (auto& index : indices) {
        auto h = terrain[index];
        sinks[index] = index;

        auto checkNeigbor = [&](std::size_t x_, std::size_t y_) {
            if (terrain[x_][y_] < h) {
                h = terrain[x_][y_];
                sinks[index] = sinks[x_][y_];
            }
        };

        auto x = index.x;
        auto y = index.y;
        if (x > 0)
            checkNeigbor(x - 1, y);
        if (x + 1 < terrain.width())
            checkNeigbor(x + 1, y);
        if (y > 0)
            checkNeigbor(x, y - 1);
        if (y + 1 < terrain.height())
            checkNeigbor(x, y + 1);
    }

    return sinks;
}

struct Basin {
    Index sink;
    std::vector<Index> indices;
    std::size_t filledTiles = 0;
    std::optional<Index> excessPoint;
    float waterLevel = 0;
    float excess = 0;
};
using BasinRef = std::reference_wrapper<Basin>;

std::optional<Index> lowestForeignNeighbor(const MapF& terrain,
                                           const Map<Index>& sinks,
                                           const Index& index)
{
    std::optional<Index> neighbor;
    auto h = std::numeric_limits<float>::max();

    auto checkNeigbor = [&](std::size_t x_, std::size_t y_) {
        if (terrain[x_][y_] < h && sinks[x_][y_] != sinks[index]) {
            h = terrain[x_][y_];
            neighbor = {x_, y_};
        }
    };

    auto x = index.x;
    auto y = index.y;
    if (x > 0)
        checkNeigbor(x - 1, y);
    if (x + 1 < terrain.width())
        checkNeigbor(x + 1, y);
    if (y > 0)
        checkNeigbor(x, y - 1);
    if (y + 1 < terrain.height())
        checkNeigbor(x, y + 1);

    return neighbor;
}

void fillBasin(const MapF& terrain, const Map<Index>& sinks, Basin& basin)
{
    float lowestNeighborHeight = std::numeric_limits<float>::max();
    if (basin.excessPoint.has_value())
        lowestNeighborHeight = terrain[basin.excessPoint.value()];
    if (lowestNeighborHeight < basin.waterLevel)
        return;

    for (auto i = basin.filledTiles; i < basin.indices.size(); ++i) {
        auto& index = basin.indices[i];
        float maxHeight = std::min(terrain[index], lowestNeighborHeight);
        float levelDiff = maxHeight - basin.waterLevel;
        float neededWater = levelDiff * basin.filledTiles;
        if (neededWater <= basin.excess) {
            basin.excess -= neededWater;
            basin.waterLevel = maxHeight;

            if (terrain[index] < lowestNeighborHeight) {
                auto neighbor = lowestForeignNeighbor(terrain, sinks, index);
                if (neighbor.has_value()) {
                    auto neighborHeight = terrain[neighbor.value()];
                    if (neighborHeight < lowestNeighborHeight) {
                        lowestNeighborHeight = neighborHeight;
                        basin.excessPoint = neighbor;
                        if (lowestNeighborHeight < basin.waterLevel) {
                            break;
                        }
                    }
                }
            }
        }
        else {
            basin.waterLevel += basin.excess / basin.filledTiles;
            basin.excess = 0;
            break;
        }
        if (terrain[index] > lowestNeighborHeight)
            break;
        else
            ++basin.filledTiles;
    }
    if (basin.filledTiles == basin.indices.size()) {
        float levelDiff = lowestNeighborHeight - basin.waterLevel;
        float neededWater = levelDiff * basin.filledTiles;
        if (neededWater <= basin.excess) {
            basin.excess -= neededWater;
            basin.waterLevel = lowestNeighborHeight;
        }
        else {
            basin.waterLevel += basin.excess / basin.filledTiles;
            basin.excess = 0;
        }
    }
}

std::map<Index, Basin> computeBasins(const MapF& terrain,
                                     const MapF& precipitation,
                                     const std::vector<Index>& sortedIndices,
                                     const Map<Index>& sinks)
{
    std::map<Index, Basin> basins;
    for (auto& index : sortedIndices)
        basins[sinks[index]].indices.push_back(index);

    for (auto& [sink, basin] : basins) {
        basin.sink = sink;
        for (auto& index : basin.indices)
            basin.excess += precipitation[index];

        basin.waterLevel = terrain[sink];
        fillBasin(terrain, sinks, basin);
    }

    return basins;
}

bool waterLevelComp(const Basin& lhs, const Basin& rhs) {
    return lhs.waterLevel > rhs.waterLevel;
}

void transferExcesses(const MapF& terrain,
                      const Map<Index>& sinks,
                      std::map<Index, Basin>& basins,
                      std::vector<BasinRef>& sortedBasins)
{
    for (auto it = sortedBasins.begin(); it != sortedBasins.end(); ++it) {
        Basin& basin = it->get();
        if (basin.excess > 0) {
            if (basin.excessPoint.has_value()) {
                Index neighbor = basin.excessPoint.value();
                Index otherSink = sinks[neighbor];
                Basin& otherBasin = basins[otherSink];
                bool update = isZero(otherBasin.excess);

                otherBasin.excess += basin.excess;
                basin.excess = 0;
                if (update) {
                    float oldWater = otherBasin.waterLevel;
                    fillBasin(terrain, sinks, otherBasin);
                    float newWater = otherBasin.waterLevel;

                    if (!isEqual(oldWater, newWater)) {
                        std::sort(it + 1, sortedBasins.end(), waterLevelComp);
                    }
                }
            }
            else {
                std::cerr << "excess without excess point at (" << basin.sink.x
                          << ", " << basin.sink.y << ")";
            }
        }
    }
}

void mergeTo(const MapF& terrain, Map<Index>& sinks,
             Basin& base, Basin& extension)
{
    base.indices.insert(base.indices.end(),
                        extension.indices.begin(),
                        extension.indices.end());
    auto comp = makeHeightComp(terrain);
    std::sort(base.indices.begin(), base.indices.end(), comp);

    for (Index index : extension.indices)
        sinks[index] = base.sink;

    base.filledTiles += extension.filledTiles;
    base.excess += extension.excess;

    base.excessPoint = std::nullopt;
    float lowestNeighborHeight = std::numeric_limits<float>::max();
    for (auto i = 0u; i < base.filledTiles; ++i) {
        auto neighbor = lowestForeignNeighbor(terrain, sinks, base.indices[i]);
        if (neighbor.has_value()) {
            auto neighborHeight = terrain[neighbor.value()];
            if (neighborHeight < lowestNeighborHeight) {
                lowestNeighborHeight = neighborHeight;
                base.excessPoint = neighbor;
            }
        }
    }
}

bool mergeBasins(const MapF& terrain, Map<Index>& sinks,
                 std::map<Index, Basin>& basins,
                 std::vector<BasinRef>& sortedBasins)
{
    bool hasChanged = false;
    for (auto it = sortedBasins.begin(); it != sortedBasins.end(); ++it) {
        Basin& basin = *it;
        if (basin.excess > 0) {
            if (basin.excessPoint.has_value()) {
                Index neighbor = basin.excessPoint.value();
                Index otherSink = sinks[neighbor];
                Basin& otherBasin = basins[otherSink];
                if (std::abs(otherBasin.waterLevel - basin.waterLevel) < 0.0001f) {                    
                    bool order = terrain[basin.sink] < terrain[otherSink];
                    Basin& base = order ? basin : otherBasin;
                    Basin& extension = order ? otherBasin : basin;

                    mergeTo(terrain, sinks, base, extension);
                    fillBasin(terrain, sinks, base);
                    basins.erase(extension.sink);

                    auto otherIt = sortedBasins.begin();
                    for (; otherIt != sortedBasins.end(); ++otherIt)
                        if (&(otherIt->get()) == &otherBasin)
                            break;

                    auto baseIt = order ? it : otherIt;
                    auto extIt = order ? otherIt : it;

                    auto dist = std::distance(sortedBasins.begin(), baseIt);
                    if (baseIt > extIt)
                        --dist;
                    sortedBasins.erase(extIt);
                    it = std::next(sortedBasins.begin(), dist);
                    std::sort(sortedBasins.begin(), baseIt + 1, waterLevelComp);

                    hasChanged = true;
                }
            }
            else {
                std::cerr << "excess without excess point at ("
                          << basin.sink.x << ", " << basin.sink.y << ")";
            }
        }
    }
    return hasChanged;
}

MapF addWater(const MapF& terrain, const MapF& precipitation, float depthThreshold)
{
    auto start = std::chrono::high_resolution_clock::now();

    auto indices = sortedByHeight(terrain);
    auto sinks = computeSinks(terrain, indices);
    auto basins = computeBasins(terrain, precipitation, indices, sinks);
    std::cout << basins.size() << " initial basins\n";

    std::vector<BasinRef> sortedBasins;
    for (auto& [sink, basin] : basins) {
        (void)sink;
        sortedBasins.push_back(basin);
    }
    std::sort(sortedBasins.begin(), sortedBasins.end(), waterLevelComp);

    int i = 0;
    bool hasChanged = true;
    while (hasChanged) {
        transferExcesses(terrain, sinks, basins, sortedBasins);
        hasChanged = mergeBasins(terrain, sinks, basins, sortedBasins);
        ++i;
    }
    std::cout << "after " << i << " rounds "  << basins.size() << " are basins left\n";

    MapF water(terrain.width(), terrain.height(), 0);
    for (auto& [sink, basin] : basins) {
        float basinDepth = basin.waterLevel - terrain[sink];
        if (basinDepth > depthThreshold)
            for (auto i = 0u; i < basin.filledTiles; ++i) {
                auto index = basin.indices[i];
                water[index] = basin.waterLevel - terrain[index];
            }
    }

    auto end = std::chrono::high_resolution_clock::now();
    using Unit = std::chrono::milliseconds;
    auto duration = std::chrono::duration_cast<Unit>(end - start);
    std::cout << "time: " << duration.count() << " ms\n";

    return water;
}

void verify(const MapF& terrain, const MapF& precipitation,
            const MapF& water)
{
    for (std::size_t x = 0; x < terrain.width(); ++x)
        for (std::size_t y = 1; y < terrain.height(); ++y) {
            float h1 = terrain[x][y - 1];
            float h2 = terrain[x][  y  ];
            float w1 = water[x][y - 1];
            float w2 = water[x][  y  ];

            if (h1 > h2 + w2) {
                if(not isZero(w1))
                    std::cerr << "water not zero at (" << x - 1 << ", " << y << ")\n";
            }
            else if (h2 > h1 + w1) {
                if(not isZero(w2))
                    std::cerr << "water not zero at (" << x << ", " << y << ")\n";
            }
            else {
                if(not isEqual(h1 + w1, h2 + w2))
                    std::cerr << "water not even at (" << x << ", " << y << ")\n";
            }
        }
    for (std::size_t x = 1; x < terrain.width(); ++x)
        for (std::size_t y = 0; y < terrain.height(); ++y) {
            float h1 = terrain[x - 1][y];
            float h2 = terrain[  x  ][y];
            float w1 = water[x - 1][y];
            float w2 = water[  x  ][y];

            if (h1 > h2 + w2) {
                if(not isZero(w1))
                    std::cerr << "water not zero at (" << x << ", " << y - 1 << ")\n";
            }
            else if (h2 > h1 + w1) {
                if(not isZero(w2))
                    std::cerr << "water not zero at (" << x << ", " << y << ")\n";
            }
            else {
                if(not isEqual(h1 + w1, h2 + w2))
                    std::cerr << "water not even at (" << x << ", " << y << ")\n";
            }
        }

    float totalHumidity = 0;
    for (float h : precipitation)
        totalHumidity += h;

    float totalWater = 0;
    for (float w : water)
        totalWater += w;

    if (not isEqual(totalHumidity, totalWater))
        std::cout << "water lost: " << totalHumidity - totalWater << '\n';
}

void advectHumidity(const Map2F& wind, const MapF& humidity, MapF& nextHumidity)
{
    float dt = .6f;
    for (std::size_t x = 0; x < humidity.width(); ++x)
        for (std::size_t y = 0; y < humidity.height(); ++y) {
            float originalX = x - dt * wind[x][y].x + humidity.width();
            float originalY = y - dt * wind[x][y].y + humidity.height();

            auto x0 = std::size_t(originalX);
            auto y0 = std::size_t(originalY);
            float u = originalX - x0;
            float v = originalY - y0;
            x0 %= humidity.width();
            y0 %= humidity.height();
            auto x1 = (x0 + 1) % humidity.width();
            auto y1 = (y0 + 1) % humidity.height();            
            float humid = 0;
            humid += (1 - u) * (1 - v) * humidity[x0][y0];
            humid += (1 - u) *    v    * humidity[x0][y1];
            humid +=    u    * (1 - v) * humidity[x1][y0];
            humid +=    u    *    v    * humidity[x1][y1];
            nextHumidity[x][y] = humid + 0.5f * (humidity[x][y] - humid);
        }
}

void advectHumidity(const Map2F& wind, MapF& humidity, int iterations)
{    
    MapF nextHumidity(humidity.width(), humidity.height());
    for (int i = 0; i < iterations / 2; ++i) {
        advectHumidity(wind, humidity, nextHumidity);
        advectHumidity(wind, nextHumidity, humidity);
    }    
}
