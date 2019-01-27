#ifndef CELL_H
#define CELL_H

#include <cstddef>
#include <vector>
#include <cmath>

/**
 * @brief The Cell class, mappings between continuous position and grid
 */
class Cell
{
public:
    Cell();
    Cell(size_t &x, size_t &y);
    std::vector<double> mapCellsToCenterPoint(const std::vector<size_t> &location_int);
    std::vector<size_t> mapContinuousPositionToCells(const std::vector<double> &location_double);
    size_t x() const;
    size_t y() const;
    double time() const;

private:
    ///time
    double m_t;
    ///x-coordinate
    size_t m_x;
    ///y-coordinate
    size_t m_y;
};

#endif // CELL_H
