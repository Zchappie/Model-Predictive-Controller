#ifndef CONSTPARAMETER_H
#define CONSTPARAMETER_H

#include <cstddef>

/*!
  This is the Constant Parameter file, which holds all the constant global variables for MPC
  */

///minimal distance between two cars
const double minDistance = 0.5;

///the width of the grid
const double cellWidth = 0.5;
const double cellWidthLower = 0.5;

///the offset of the cell index, the number of cells in x and y direction
const size_t offsetX = 10;
const size_t offsetY = 10;

///prediction horizon
const size_t horizon=7;

///control vector size
const size_t controlVecSize = 2;

///total degree of freedom
const size_t doF = 2;

///the length of dimension(x, y, theta)
const size_t coordLength = 3;

///lambda at stage cost
const double lambda = 0.2;

///sampling instance
const double sampleInterval = 0.5;

///the upper/lower bound of the control(linear/angular velocity)
/// unit (meter/sec) and (radian/sec)
const double controlUpperBound = 0.5;
const double controlLowerBound = -0.5;

///the limitation of the combination of two controls(linear and angular velocities),
///simple speaking, it's the control bound to the power of 2, as one velocity effects the other
const double controlConstraintLimit = 0.25;

#endif // CONSTPARAMETERS_H
