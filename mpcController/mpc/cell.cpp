#include "cell.h"
#include "constparameter.h"

Cell::Cell() :
    m_t(0.0),
    m_x(0),
    m_y(0)
{ }

/**
 * @brief Cell::Cell, constructor of cell class
 * @param x, cell index(unsigned integer)
 * @param y, cell index(unsigned integer)
 */
Cell::Cell(size_t &x, size_t &y):
    m_t(0.0),
    m_x(x),
    m_y(y)
{ }


size_t Cell::x() const
{
    return m_x;
}

size_t Cell::y() const
{
    return m_y;
}

double Cell::time() const
{
    return m_t;
}

/**
 * @brief Cell::mapCellsToContinuousPosition, maps grid index to the center of the grid
 * @param location_int 1D vector of the grid index pointed out which cell the robot is in
 * @return centerPoint 1D vector of center point of the cell
 */
std::vector<double> Cell::mapCellsToCenterPoint(const std::vector<size_t> &location_int)
{
    ///cellWidth/offsetX/offsetY in constparameters.h, which is a constant parameter
    std::vector<double> centerPoint(location_int.size());
    for(size_t i=0; i<location_int.size();i=i+2)
    {
        centerPoint.at(i) = cellWidth*(location_int.at(i)+0.5)-offsetX;
        centerPoint.at(i+1) = cellWidth*(location_int.at(i+1)+0.5)-offsetY;
    }
    return centerPoint;
}

/**
 * @brief Cell::mapContinuousPositionToCells map continuous position to grid index
 * @param location_double 1D vector, only contains x, y (no angular), the location of the robot
 * @return location_int 1D vector, the cell index of where the robot locates in
 */
std::vector<size_t> Cell::mapContinuousPositionToCells(const std::vector<double> &location_double)
{
    std::vector<size_t> location_int(location_double.size());
    for(size_t i=0; i<location_double.size();i=i+2)
    {
        location_int[i] = std::min((double)offsetX - 1.0, std::floor(location_double[i]/cellWidth));
        location_int[i+1] = std::min((double)offsetY - 1.0, std::floor(location_double[i+1]/cellWidth));
    }
    return location_int;
}


