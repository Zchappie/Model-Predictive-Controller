#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "vectorhelper.h"
#include "model.h"

#include <algorithm>
#include <vector>

#include <QtCore/QDataStream>
#include <QDebug>


/**
 * @brief The CostFunction class, calculate the state cost, and be ready to send data to NLopt
 */
class CostFunction
{
public:
    CostFunction();
    CostFunction(const std::vector<double> &x0, const std::vector<double> &target, const double& t0);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    static double wrapCostFunctionObject(const std::vector<double> &u, std::vector<double>& grad, void* data);
    double getStageCosts(const std::vector<double>& x, const std::vector<double>& u) const;

    friend QDataStream& operator<<(QDataStream& out, const CostFunction& costFunction);
    friend QDataStream& operator>>(QDataStream& in, CostFunction& costFunction);

private:
    ///start value
    std::vector<double> m_x0;
    ///target
    std::vector<double> m_target;
    ///start time
    double m_t0;
    ///sampling step
    double m_T;
    ///horizon length
    size_t m_N;
    ///lambda value
    double m_lambda;

};



#endif // COSTFUNCTION_H
