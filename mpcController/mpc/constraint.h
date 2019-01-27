#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "model.h"
#include "cell.h"
#include "vectorhelper.h"

#include <cstddef>
#include <vector>
#include <math.h>
#include <algorithm>
#include <memory>
#include <QtGlobal>
#include <QDebug>

#include <QtCore/QDataStream>

/**
 * @brief The Constraint class, prepare the constraint for a referred car
 *
 */
class Constraint
{
public:
    Constraint();
    Constraint(const double& t, const std::vector<double> &selfPos);
    static double wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    double constraintTime() const;
    void setActualSystem(const std::shared_ptr<Model> &model, const double& t0);

    friend QDataStream& operator<<(QDataStream& out, const Constraint& constraint);
    friend QDataStream& operator>>(QDataStream& in, Constraint& constraint);

private:
    ///current state
    std::vector<double> m_x0;
    ///current global time
    double m_t0;    
    ///time instant, which is valid for the constraint
    double m_t;
    ///horizon length
    size_t m_N;
    ///sample interval
    double m_T;
    std::shared_ptr<Model> m_model;
    ///blocked cell by another car
    std::vector<double> m_selfPos;

    double distanceNorm(const std::vector<double> &state, const std::vector<double> &blocked_c);
    double psiQ();
};


#endif // CONSTRAINT_H
