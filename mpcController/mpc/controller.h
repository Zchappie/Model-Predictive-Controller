#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "worker.h"
#include "mpccontroller.h"

#include <QObject>
#include <QThread>

#include <vector>

/**
 * @brief The Controller class makes a copy of "need to be optimized" problem's parameters,
 * then intialize the Worker class which takes a copy of the MPC controller to perform the optimization in QThread
 */
class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(const std::shared_ptr<MPCController>& controller, const std::vector<std::vector<double> >& control, const std::vector<Constraint> &constraints,
               const std::vector<ControlConstraint> &controlConstraints, const std::vector<double>& x0, const std::vector<double>& target, const double& t0);
    ~Controller();

public slots:
    void start();
    void handleResults(const std::vector<std::vector<double> > &result);

signals:
    void finished(const std::vector<std::vector<double> > &);

private:
    ///result of the optimisation
    std::vector<std::vector<double> > m_result;
    ///worker thread to handle the optimisation
    QPointer<Worker> worker;
};

#endif // CONTROLLER_H
