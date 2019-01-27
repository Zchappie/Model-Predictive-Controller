#include "controller.h"
#include "robot.h"

/**
 * @brief Controller::Controller hols the instance to start the worker on a seperate thread
 * @param controller copies the given instance of the mpc controller
 * @param control initial control guess
 * @param constraints local constraints (other robots, obstacles)
 * @param controlConstraints (control constraints)
 * @param x0 start value
 * @param target target to reach
 * @param t0 global time
 */
Controller::Controller(const std::shared_ptr<MPCController>& controller, const std::vector<std::vector<double> > &control,
                       const std::vector<Constraint> &constraints, const std::vector<ControlConstraint> &controlConstraints,
                       const std::vector<double> &x0, const std::vector<double> &target, const double &t0)
{
    worker = new Worker(controller, control, constraints, controlConstraints, x0, target, t0);
    connect(worker, SIGNAL(resultReady(const std::vector<std::vector<double> > &)), this, SLOT(handleResults(const std::vector<std::vector<double> > &)));
}

/**
 * @brief Controller::start starts the optimisation
 */
void Controller::start()
{
    worker->start();
}

Controller::~Controller()
{
    worker->quit();
    worker->wait();
}

/**
 * @brief Controller::handleResults is called when optimisation is finished, signal finished is emitted
 * @param result
 */
void Controller::handleResults(const std::vector<std::vector<double> > &result)
{
    qDebug()<<"[Controller] here now";
    m_result = result;
    emit finished(m_result);
}

