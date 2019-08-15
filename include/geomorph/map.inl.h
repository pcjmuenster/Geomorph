#include "map.h"

template <class T>
Map<T>::Map(std::size_t width, std::size_t height, T value)
    : values_(width, Column(height, value))
{
}

template <class T>
void Map<T>::resize(std::size_t width, std::size_t height)
{
    values_.resize(width);
    for (Column& c : values_)
        c.resize(height);
}

template <class T>
Map<T>::iter::iter(Map* map, std::size_t x, std::size_t y)
    : map_{map}, x_{x}, y_{y}
{
}

template <class T>
bool Map<T>::iter::operator==(const Map::iter& rhs) const
{
    return map_ == rhs.map_ && x_ == rhs.x_ && y_ == rhs.y_;
}

template <class T>
bool Map<T>::iter::operator!=(const Map::iter& rhs) const
{
    return !(*this == rhs);
}

template <class T>
typename Map<T>::iter& Map<T>::iter::operator++()
{    
    if (y_ == map_->height() - 1) {
        y_ = 0;
        ++x_;
    }
    else {
        ++y_;
    }    
    return *this;
}

template <class T>
Map<T>::const_iter::const_iter(const Map* map, std::size_t x, std::size_t y)
    : map_{map}, x_{x}, y_{y}
{
}

template <class T>
bool Map<T>::const_iter::operator==(const Map::const_iter& rhs) const
{
    return map_ == rhs.map_ && x_ == rhs.x_ && y_ == rhs.y_;
}

template <class T>
bool Map<T>::const_iter::operator!=(const Map::const_iter& rhs) const
{
    return !(*this == rhs);
}

template <class T>
typename Map<T>::const_iter& Map<T>::const_iter::operator++()
{
    if (y_ == map_->height() - 1) {
        y_ = 0;
        ++x_;
    }
    else {
        ++y_;
    }
    return *this;
}



