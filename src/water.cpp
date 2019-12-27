#include "geomorph/water.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <tuple>

#include "geomorph/map.h"
#include "geomorph/math_utils.h"
#include "geomorph/vec2.h"

void diffuse_water(const DMap& terrain, DMap& water, std::size_t x, std::size_t y)
{
    double total_water = 0;
    total_water += water[  x  ][  y  ];
    total_water += water[  x  ][y + 1];
    total_water += water[x + 1][  y  ];
    total_water += water[x + 1][y + 1];
    if (is_zero(total_water))
        return;

    double heights[4] {
        terrain[  x  ][  y  ],
        terrain[  x  ][y + 1],
        terrain[x + 1][  y  ],
        terrain[x + 1][y + 1]
    };
    std::sort(heights, heights + 4);
    double level = heights[0];
    int tiles = 1;
    for (int i = 1; i < 4; ++i) {
        double needed_water = (heights[i] - level) * i;
        if (needed_water < total_water) {
            total_water -= needed_water;
            level = heights[i];
            ++tiles;
        }
        else break;
    }
    level += total_water / tiles;
    auto rise_water = [&water, &terrain, level](std::size_t x, std::size_t y){
        water[x][y] = terrain[x][y] < level ? level - terrain[x][y] : 0;
    };
    rise_water(  x  ,   y  );
    rise_water(  x  , y + 1);
    rise_water(x + 1,   y  );
    rise_water(x + 1, y + 1);
}

void diffuse_water(const DMap& terrain, DMap& water, int iterations)
{
    DMap old_water = water;

    auto width = terrain.width();
    auto height = terrain.height();
    for (int n = 0; n < iterations; ++n) {
        for (auto x = 0u; x < width - 1; ++x)
            for (auto y = 0u; y < height - 1; ++y) {
                diffuse_water(terrain, water, x, y);
            }
        for (auto y = height - 1; y > 0 ; --y)
            for (auto x = width - 1; x > 0; --x) {
                diffuse_water(terrain, water, x - 1, y - 1);
            }
    }

    double max = 0;
    for (auto x = 0u; x < width - 1; ++x)
        for (auto y = 0u; y < height - 1; ++y) {
            double diff = std::abs(water[x][y] - old_water[x][y]);
            if (diff > max)
                max = diff;
        }
}

auto make_height_comp(const DMap& map)
{
    return [&map](const Index& lhs, const Index& rhs) {
        return std::make_tuple(map[lhs], lhs) < std::make_tuple(map[rhs], rhs);
    };
}

std::vector<Index> sorted_by_height(const DMap& terrain)
{
    auto comp = make_height_comp(terrain);

    std::vector<Index> indices;
    indices.reserve(terrain.width() * terrain.height());
    for (auto x = 0u; x < terrain.width(); ++x)
        for (auto y = 0u; y < terrain.height(); ++y)
            indices.push_back({x, y});
    std::sort(indices.begin(), indices.end(), comp);

    return indices;
}

template <class Func>
void for_each_neighbor(const Index& index, const Size& size, Func func)
{
    auto x = index.x;
    auto y = index.y;
    bool lft = x > 0, rgt = x + 1 < size.width;
    bool bot = y > 0, top = y + 1 < size.height;

    if (lft)
        func(x - 1, y);
    if (rgt)
        func(x + 1, y);
    if (bot)
        func(x, y - 1);
    if (top)
        func(x, y + 1);

//    if (lft && bot)
//        func(x - 1, y - 1);
//    if (lft && top)
//        func(x - 1, y + 1);
//    if (rgt && bot)
//        func(x + 1, y - 1);
//    if (rgt && top)
//        func(x + 1, y + 1);
}

Map<Index> compute_sinks(const DMap& terrain, const std::vector<Index>& indices)
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

        for_each_neighbor(index, terrain.size(), checkNeigbor);
    }

    return sinks;
}

struct Basin {
    Index sink;
    std::vector<Index> indices;
    std::size_t filled_tiles = 0;
    std::optional<Index> excess_point;
    double water_level = 0;
    double excess = 0;
};
using BasinRef = std::reference_wrapper<Basin>;

std::optional<Index>
lowest_foreign_neighbor(const DMap& terrain, const Map<Index>& sinks,
                        const Index& index)
{
    std::optional<Index> neighbor;
    auto h = std::numeric_limits<double>::max();

    auto check_neighbor = [&](std::size_t x_, std::size_t y_) {
        if (terrain[x_][y_] < h && sinks[x_][y_] != sinks[index]) {
            h = terrain[x_][y_];
            neighbor = {x_, y_};
        }
    };
    for_each_neighbor(index, terrain.size(), check_neighbor);

    return neighbor;
}

void fill_basin(const DMap& terrain, const Map<Index>& sinks, Basin& basin)
{
    double lowest_neighbor_height = std::numeric_limits<double>::max();
    if (basin.excess_point.has_value())
        lowest_neighbor_height = terrain[basin.excess_point.value()];
    if (lowest_neighbor_height < basin.water_level)
        return;

    for (auto i = basin.filled_tiles; i < basin.indices.size(); ++i) {
        auto& index = basin.indices[i];
        double max_height = std::min(terrain[index], lowest_neighbor_height);
        double level_diff = max_height - basin.water_level;
        double needed_water = level_diff * basin.filled_tiles;
        if (needed_water <= basin.excess) {
            basin.excess -= needed_water;
            basin.water_level = max_height;

            if (terrain[index] < lowest_neighbor_height) {
                auto neighbor = lowest_foreign_neighbor(terrain, sinks, index);
                if (neighbor.has_value()) {
                    auto neighbor_height = terrain[neighbor.value()];
                    if (neighbor_height < lowest_neighbor_height) {
                        lowest_neighbor_height = neighbor_height;
                        basin.excess_point = neighbor;
                        if (lowest_neighbor_height < basin.water_level) {
                            break;
                        }
                    }
                }
            }
        }
        else {
            basin.water_level += basin.excess / basin.filled_tiles;
            basin.excess = 0;
            break;
        }
        if (terrain[index] > lowest_neighbor_height)
            break;
        else
            ++basin.filled_tiles;
    }
    if (basin.filled_tiles == basin.indices.size()) {
        double level_diff = lowest_neighbor_height - basin.water_level;
        double needed_water = level_diff * basin.filled_tiles;
        if (needed_water <= basin.excess) {
            basin.excess -= needed_water;
            basin.water_level = lowest_neighbor_height;
        }
        else {
            basin.water_level += basin.excess / basin.filled_tiles;
            basin.excess = 0;
        }
    }
}

std::map<Index, Basin>
compute_basins(const DMap& terrain, const DMap& precipitation,
               const std::vector<Index>& sorted_indices, const Map<Index>& sinks)
{
    std::map<Index, Basin> basins;
    for (auto& index : sorted_indices)
        basins[sinks[index]].indices.push_back(index);

    for (auto& [sink, basin] : basins) {
        basin.sink = sink;
        for (auto& index : basin.indices)
            basin.excess += precipitation[index];

        basin.water_level = terrain[sink];
        fill_basin(terrain, sinks, basin);
    }

    return basins;
}

bool water_level_comp(const Basin& lhs, const Basin& rhs)
{
    return lhs.water_level > rhs.water_level;
}

void transfer_excesses(const DMap& terrain, DMap& flowing_water,
                       const Map<Index>& sinks, std::map<Index, Basin>& basins,
                       std::vector<BasinRef>& sorted_basins)
{
    for (auto it = sorted_basins.begin(); it != sorted_basins.end(); ++it) {
        Basin& basin = it->get();
        if (basin.excess > 0) {
            if (basin.excess_point.has_value()) {
                Index neighbor = basin.excess_point.value();
                Index other_sink = sinks[neighbor];
                Basin& other_basin = basins[other_sink];
                bool update = is_zero(other_basin.excess);

                other_basin.excess += basin.excess;
                flowing_water[neighbor] += basin.excess;
                basin.excess = 0;
                if (update) {
                    auto [first, last]
                            = std::equal_range(sorted_basins.begin(),
                                               sorted_basins.end(),
                                               other_basin, water_level_comp);
                    auto other_it = first;
                    for (; other_it != last; ++other_it)
                        if (&(other_it->get()) == &other_basin)
                            break;

                    double old_water = other_basin.water_level;
                    fill_basin(terrain, sinks, other_basin);
                    double new_water = other_basin.water_level;

                    if (new_water > old_water) {
                        auto new_other_it
                                = std::lower_bound(sorted_basins.begin(),
                                                   sorted_basins.end(),
                                                   other_basin, water_level_comp);
                        if (new_other_it < it && it < other_it)
                            ++it;
                        std::copy_backward(new_other_it, other_it, other_it + 1);
                        *new_other_it = other_basin;
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

void merge(const DMap& terrain, Map<Index>& sinks,
           Basin& base, Basin& extension)
{
    base.indices.insert(base.indices.end(),
                        extension.indices.begin(),
                        extension.indices.end());
    auto comp = make_height_comp(terrain);
    std::sort(base.indices.begin(), base.indices.end(), comp);

    for (Index index : extension.indices)
        sinks[index] = base.sink;

    base.filled_tiles += extension.filled_tiles;
    base.excess += extension.excess;

    base.excess_point = std::nullopt;
    double lowest_neighbor_height = std::numeric_limits<double>::max();
    for (auto i = 0u; i < base.filled_tiles; ++i) {
        auto neighbor = lowest_foreign_neighbor(terrain, sinks, base.indices[i]);
        if (neighbor.has_value()) {
            auto neighbor_height = terrain[neighbor.value()];
            if (neighbor_height < lowest_neighbor_height) {
                lowest_neighbor_height = neighbor_height;
                base.excess_point = neighbor;
            }
        }
    }
}

using Iter = std::vector<BasinRef>::iterator;
void reorder_basins(std::vector<BasinRef>& sorted_basins, Iter base_it, Iter ext_it)
{
    Basin& base = *base_it;
    auto new_base_it = std::lower_bound(sorted_basins.begin(),
                                      sorted_basins.end(),
                                      base, water_level_comp);
    if (base_it < ext_it) {
        std::copy_backward(new_base_it, base_it, base_it + 1);
        sorted_basins.erase(ext_it);
    }
    else {
        std::copy_backward(new_base_it, ext_it, ext_it + 1);
        sorted_basins.erase(base_it);
    }
    *new_base_it = base;
}

bool merge_basins(const DMap& terrain, Map<Index>& sinks,
                  std::map<Index, Basin>& basins, std::vector<BasinRef>& sorted_basins)
{
    bool has_changed = false;
    for (auto it = sorted_basins.begin(); it != sorted_basins.end(); ++it) {
        Basin& basin = *it;
        if (basin.excess > 0) {
            if (basin.excess_point.has_value()) {
                Index neighbor = basin.excess_point.value();
                Index other_sink = sinks[neighbor];
                Basin& other_basin = basins[other_sink];

                if (is_equal(other_basin.water_level, basin.water_level)) {
                    auto [first, last]
                            = std::equal_range(sorted_basins.begin(),
                                               sorted_basins.end(),
                                               other_basin, water_level_comp);
                    auto other_it = first;
                    for (; other_it != last; ++other_it)
                        if (&(other_it->get()) == &other_basin)
                            break;

                    bool order = terrain[basin.sink] < terrain[other_sink];
                    Basin& base = order ? basin : other_basin;
                    Basin& extension = order ? other_basin : basin;
                    merge(terrain, sinks, base, extension);
                    fill_basin(terrain, sinks, base);
                    basins.erase(extension.sink);

                    auto base_it = order ? it : other_it;
                    auto ext_it = order ? other_it : it;
                    if (ext_it < it)
                        --it;

                    reorder_basins(sorted_basins, base_it, ext_it);

                    if (it == sorted_basins.end())
                        return true;

                    has_changed = true;
                }
            }
            else {
                std::cerr << "excess without excess point at ("
                          << basin.sink.x << ", " << basin.sink.y << ")";
            }
        }
    }
    return has_changed;
}

std::optional<Index>
lowest_neighbor_by_total_height(const DMap& terrain, const DMap& water, Index index)
{
    auto total_height = [&] (Index index) {
        return terrain[index] + water[index];
    };

    std::optional<Index> neighbor;
    double min = total_height(index);
    auto check_neigbor = [&](std::size_t x, std::size_t y) {
        Index nb = {x, y};
        if (total_height(nb) < min) {
            min = total_height(nb);
            neighbor = nb;
        }
    };
    for_each_neighbor(index, terrain.size(), check_neigbor);

    return neighbor;
}

DMap stationary_water(const DMap& terrain, const std::map<Index, Basin>& basins,
                      double depth_threshold)
{
    DMap stationary_water(terrain.size(), 0);
    for (auto& [sink, basin] : basins) {
        double basin_depth = basin.water_level - terrain[sink];
        if (basin_depth > depth_threshold)
            for (auto i = 0u; i < basin.filled_tiles; ++i) {
                auto index = basin.indices[i];
                stationary_water[index] = basin.water_level - terrain[index];
            }
    }
    return stationary_water;
}

DMap make_water(const DMap& terrain, const DMap& precipitation, double depth_threshold)
{
    auto indices = sorted_by_height(terrain);
    auto sinks = compute_sinks(terrain, indices);
    auto basins = compute_basins(terrain, precipitation, indices, sinks);

    std::clog << basins.size() << " initial basins\n";

    std::vector<BasinRef> sorted_basins;
    for (auto& [sink, basin] : basins) {
        (void)sink;
        sorted_basins.push_back(basin);
    }
    std::sort(sorted_basins.begin(), sorted_basins.end(), water_level_comp);

    DMap flowing_water = precipitation;

    int i = 0;
    for (bool hasChanged = true; hasChanged; ++i) {
        transfer_excesses(terrain, flowing_water, sinks, basins, sorted_basins);
        hasChanged = merge_basins(terrain, sinks, basins, sorted_basins);
    }
    std::clog << "after " << i << " rounds " << basins.size() << " basins are left\n";

    (void)flowing_water;  // unused

    return stationary_water(terrain, basins, depth_threshold);
}

void flow_downhill(const DMap& terrain, const DMap& stationary, DMap& flowing_water)
{
    std::vector<Index> dry_indices;
    for (std::size_t x = 0; x < terrain.width(); ++x)
        for (std::size_t y = 0; y < terrain.height(); ++y)
            if (is_zero(stationary[x][y]))
                dry_indices.emplace_back(x, y);
            else
                flowing_water[x][y] = 0;

    std::sort(dry_indices.rbegin(), dry_indices.rend(), make_height_comp(terrain));

    for (auto index : dry_indices) {
        auto neighbor = lowest_neighbor_by_total_height(terrain, stationary, index);
        if (neighbor && is_zero(stationary[neighbor.value()])) {
            flowing_water[neighbor.value()] += flowing_water[index];
        }
    }
}

std::pair<DMap, DMap>
make_water_with_flow(const DMap& terrain, const DMap& precipitation,
                     double depth_threshold)
{
    auto indices = sorted_by_height(terrain);
    auto sinks = compute_sinks(terrain, indices);
    auto basins = compute_basins(terrain, precipitation, indices, sinks);

    std::clog << basins.size() << " initial basins\n";

    std::vector<BasinRef> sorted_basins;
    for (auto& [sink, basin] : basins) {
        (void)sink;
        sorted_basins.push_back(basin);
    }
    std::sort(sorted_basins.begin(), sorted_basins.end(), water_level_comp);

    DMap flowing_water = precipitation;

    int i = 0;
    for (bool has_changed = true; has_changed; ++i) {
        transfer_excesses(terrain, flowing_water, sinks, basins, sorted_basins);
        has_changed = merge_basins(terrain, sinks, basins, sorted_basins);
    }

    DMap stationary = stationary_water(terrain, basins, depth_threshold);
    flow_downhill(terrain, stationary, flowing_water);

    std::clog << "after " << i << " rounds " << basins.size() << " basins are left\n";

    return {stationary, flowing_water};
}

std::pair<Index, double>
lowest_moore_neighbor(const DMap& total_height, const Index& c)
{
    Index lowest_neighbor;
    double lowest = std::numeric_limits<double>::max();
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j) {
            if (i == 0 && j == 0)
                continue;
            auto x = std::size_t(int(c.x) + i);
            auto y = std::size_t(int(c.y) + j);
            if (x < total_height.width() && y < total_height.height()) {
                double t = total_height[x][y];
                if (t < lowest) {
                    lowest_neighbor = {x, y};
                    lowest = t;
                }
            }
        }
    return {lowest_neighbor, lowest};
};

Map<int>
strahler_numbers(const DMap& total_height, std::vector<Index> sorted_by_total_height)
{
    Map<int> strahler_numbers(total_height.width(), total_height.height(), 1);
    Map<int> max_prev_strahler(total_height.width(), total_height.height(), 0);
    for (auto it = sorted_by_total_height.rbegin();
            it != sorted_by_total_height.rend(); ++it) {
        const Index& curr_index  = *it;
        auto [lowest_index, lowest_height] = lowest_moore_neighbor(total_height, curr_index);
        if (lowest_height < total_height[curr_index]) {
            auto& prev_strahler = max_prev_strahler[lowest_index];
            auto curr_strahler = max_prev_strahler[curr_index];
            if (prev_strahler < curr_strahler) {
                prev_strahler = curr_strahler;
                strahler_numbers[lowest_index] = curr_strahler;
            }
            else if (prev_strahler == curr_strahler) {
                strahler_numbers[lowest_index] = curr_strahler + 1;
            }
        }
    }
    return strahler_numbers;
}

void add_rivers(DMap& terrain, DMap& water)
{
    auto width = terrain.width();
    auto height = terrain.height();
    DMap total_height(width, height);
    for (std::size_t x = 0; x < terrain.width(); ++x)
        for (std::size_t y = 0; y < terrain.height(); ++y)
            total_height[x][y] = terrain[x][y] + water[x][y];

    auto sorted_by_total_height = sorted_by_height(total_height);
    auto strahler_map = strahler_numbers(total_height, sorted_by_total_height);

    auto minmax = std::minmax_element(strahler_map.begin(), strahler_map.end());
    double min = *minmax.first;
    double max = *minmax.second;
    for (std::size_t x = 0; x < strahler_map.width(); ++x)
        for (std::size_t y = 0; y < strahler_map.height(); ++y) {
            if (water[x][y] > 0)
                continue;
            double river_depth = double(strahler_map[x][y] - min) / (max - min);
            if (river_depth > 0.36) {
                river_depth = std::pow(river_depth - 0.08, 0.3) * 0.06;
                terrain[x][y] -= river_depth;
                water[x][y] = river_depth;
            }
        }
}

void advect_humidity(const D2Map& wind, const DMap& humidity, DMap& next_humidity)
{
    double dt = .6;
    for (std::size_t x = 0; x < humidity.width(); ++x)
        for (std::size_t y = 0; y < humidity.height(); ++y) {
            double original_x = x - dt * wind[x][y].x + humidity.width();
            double original_y = y - dt * wind[x][y].y + humidity.height();

            auto x0 = std::size_t(original_x);
            auto y0 = std::size_t(original_y);
            double u = original_x - x0;
            double v = original_y - y0;
            x0 %= humidity.width();
            y0 %= humidity.height();
            auto x1 = (x0 + 1) % humidity.width();
            auto y1 = (y0 + 1) % humidity.height();
            double humid = 0;
            humid += (1 - u) * (1 - v) * humidity[x0][y0];
            humid += (1 - u) *    v    * humidity[x0][y1];
            humid +=    u    * (1 - v) * humidity[x1][y0];
            humid +=    u    *    v    * humidity[x1][y1];
            next_humidity[x][y] = humid + 0.5 * (humidity[x][y] - humid);
        }
}

void advect_humidity(const D2Map& wind, DMap& humidity, int iterations)
{
    DMap next_humidity(humidity.width(), humidity.height());
    for (int i = 0; i < iterations / 2; ++i) {
        advect_humidity(wind, humidity, next_humidity);
        advect_humidity(wind, next_humidity, humidity);
    }
}


