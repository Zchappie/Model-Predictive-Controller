#include "worker.h"

/**
 * @brief Worker::Worker creates the variables on the stack, which are later used in the working thread
 * @param controller copies the given instance of the mpc controller
 * @param control initial control guess
 * @param constraints local constraints (other robots, obstacles)
 * @param controlConstraints (control constraints)
 * @param x0 start value
 * @param target target to reach
 * @param t0 global time
 */
Worker::Worker(const std::shared_ptr<MPCController>& controller, const std::vector<std::vector<double> >& control,
               const std::vector<Constraint>& constraints, const std::vector<ControlConstraint>& controlConstraints, const std::vector<double>& x0, const std::vector<double>& target, const double& t0) :
    m_pc(*(controller.get())),
    m_control(control),
    m_constraints(constraints),
    m_controlConstraints(controlConstraints),
    m_x0(x0),
    m_target(target),
    m_t0(t0)
{}

/**
 * @brief Worker::run, do the optimization and send out
 */
void Worker::run()
{
    this->setObjectName("Worker thread");
    qDebug()<<"[Worker] NAME:"<<this->currentThreadId();
    qDebug() << "[Worker] calculating.." << endl;
    m_pc.initSocketClient();
    std::vector<std::vector<double> > controls = m_pc.optimize(m_control, m_constraints, m_controlConstraints, m_x0, m_target, m_t0);
    emit resultReady(controls);
}
