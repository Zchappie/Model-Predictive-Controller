#ifndef TESTOBSTACLE_H
#define TESTOBSTACLE_H

#include "model.h"
#include "cell.h"
#include "constparameter.h"
#include "vectorhelper.h"


#include <cstddef>
#include <vector>
#include <math.h>
#include <algorithm>
#include <memory>
#include <QtGlobal>
#include <QDebug>

/**
 * @brief The TestObstacle class to test the single car optimization after add an obstacle
 */
class TestObstacle
{
public:
    TestObstacle(const double& t);
    std::vector<std::vector<double> > calcTraj(const std::vector<double>& control);
    static double wrapTestObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &control, std::vector<double> &grad, void* f_data = nullptr);
    void setActual(const double& t0);

private:
    std::shared_ptr<Model> m_model;
    size_t m_N;
    ///current global time
    double m_t0;
    ///time instant, which is valid for the constraint
    double m_t;
    ///sample interval
    double m_T;
};


#endif // TESTOBSTACLE_H
