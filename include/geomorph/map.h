#ifndef MAP_H
#define MAP_H

#include <tuple>
#include <vector>

#include "map_fwd.h"

/**
 * @brief represents an index into a two-dimensional array
 *        or a Map
 */
struct Index {
    Index() = default;
    Index(std::size_t x, std::size_t y) : x{x}, y{y} {}
    std::size_t x, y;

    bool operator==(const Index& rhs) const {
        return std::make_tuple(x, y) == std::make_tuple(rhs.x, rhs.y);
    }
    bool operator!=(const Index& rhs) const {
        return !(*this == rhs);
    }
    bool operator<(const Index& rhs) const {
        return std::make_tuple(x, y) < std::make_tuple(rhs.x, rhs.y);
    }
};

struct Size {
    std::size_t width;
    std::size_t height;
};

/**
 * @brief is a two-dimensional array whose elements are of type T
 */
template <class T>
class Map
{
    using Column = std::vector<T>;
    using Field = std::vector<Column>;

public:
    Map() : Map(0, 0) {}
    Map(Size size, T value = T()) : Map{size.width, size.height, value} {}
    Map(std::size_t width, std::size_t height, T value = T());

    auto& operator[](const Index& index) { return values_[index.x][index.y]; }
    auto operator[](const Index& index) const { return values_[index.x][index.y]; }
    Column& operator[](std::size_t i) { return values_[i]; }
    const Column& operator[](std::size_t i) const { return values_[i]; }

    auto width() const { return values_.size(); }
    auto height() const { return values_.empty() ? 0 : values_[0].size(); }
    Size size() const { return {width(), height()}; }

    void resize(std::size_t width, std::size_t height);

    class iter {
    public:
        iter() = default;
        iter(Map* map, std::size_t x, std::size_t y);

        bool operator==(const iter& rhs) const;
        bool operator!=(const iter& rhs) const;
        auto& operator*() { return (*map_)[x_][y_]; }
        auto operator*() const { return (*map_)[x_][y_]; }
        iter& operator++();

    private:
        Map* map_;
        std::size_t x_;
        std::size_t y_;
    };

    iter begin() { return {this, 0, 0}; }
    iter end() { return {this, width(), 0}; }


    class const_iter {
    public:
        const_iter() = default;
        const_iter(const Map* map, std::size_t x, std::size_t y);

        bool operator==(const const_iter& rhs) const;
        bool operator!=(const const_iter& rhs) const;
        auto operator*() const { return (*map_)[x_][y_]; }
        const_iter& operator++();

    private:
        const Map* map_;
        std::size_t x_;
        std::size_t y_;
    };

    const_iter begin() const { return {this, 0, 0}; }
    const_iter end() const { return {this, width(), 0}; }
private:
    Field values_;
};

#include "map.inl.h"

#endif // MAP_H
