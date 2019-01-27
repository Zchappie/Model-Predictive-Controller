#ifndef WORKER_H
#define WORKER_H

#include "robot.h"
#include "mpccontroller.h"
#include "constraint.h"
#include "controlconstraint.h"

#include <QtCore/QObject>
#include <QThread>
#include <vector>

/**
 * @brief The Worker class starts a QThread to run the optimisation to retain the robot's main event loop to receive messages
 */
class Worker : public QThread
{
    Q_OBJECT
public:
    Worker(const std::shared_ptr<MPCController>& controller, const std::vector<std::vector<double> > &control,
           const std::vector<Constraint> &constraints, const std::vector<ControlConstraint> &controlConstraints,
           const std::vector<double>& x0, const std::vector<double>& target, const double& t0);
    void run();

signals:
    ///send out the result when the calculation is done
    void resultReady(const std::vector<std::vector<double> > &result);

private:
    ///local copy of the MPC-controller
    MPCController m_pc;
    ///initial control guess
    std::vector<std::vector<double> > m_control;
    ///constraints (other robots, obstacles)
    std::vector<Constraint> m_constraints;
    ///control constraints(U)
    std::vector<ControlConstraint> m_controlConstraints;
    ///current start
    std::vector<double> m_x0;
    ///target position
    std::vector<double> m_target;
    ///current global time
    double m_t0;
};

#endif // WORKER_H
