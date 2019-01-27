#include "mpccontroller.h"
#include "CostFunction.h"

#include <iostream>

/**
 * @brief MPCController::MPCController
 * @param car id of robot
 * @param N horizon length
 * @param T sampling step size (discretisation)
 * @param lambda (fraction for cost function to impose control value)
 */
MPCController::MPCController(const std::string &car) :
    m_car(car)
{
    qDebug("[MPCController] Initializing the MPCController class...");
    m_N = horizon;
    m_T = sampleInterval;
    m_lambda = lambda;
    m_model = std::make_shared<Model>();
}

void MPCController::addConstraints(const std::vector<Constraint>& constraints)
{
    m_constraints = constraints;
}

void MPCController::addControlConstraint(const std::vector<ControlConstraint>& controlConstraints)
{
    m_controlConstraints = controlConstraints;
}

/**
 * @brief optimize calculates the next prediction for a given control vector as initial start
 * @param control could be filled up with 0 or the shifted prediction from the last time instant over the full horizon \f$ u = (u_1, u_2, u_3, ..., u_{N-1})\f$
 * @param x0 start position for the prediction
 * @param target target where the robot should go
 * @param t0 start time
 * @param testObstacles, test purpose for adding single obstacle, after used, comment out " const std::vector<TestObstacle> &testObstacles"
 * @return
 */
std::vector<std::vector<double> > MPCController::optimize(const std::vector<std::vector<double> >& control, const std::vector<Constraint>& constraints,
                                                          const std::vector<ControlConstraint> &controlConstraints, const std::vector<double>& x0,
                                                          const std::vector<double>& target, const double& t0)
{
    m_constraints = constraints;
    m_controlConstraints = controlConstraints;
    m_model->setStart(x0);

    ///initialise the nonlinear optimiser, LN_COBYLA is the derivative-free algoristhm: local optimum no derivative
    std::vector<double> oneDimVector = VectorHelper::reshapeXdTo1d(control);
    nlopt::opt optimizer(nlopt::LN_COBYLA, oneDimVector.size());
    ///set the cost function with start, target and other parameters
    CostFunction costFunction(x0, target, t0);
    optimizer.set_min_objective(CostFunction::wrapCostFunctionObject, &costFunction);

    ///set the bound of the linear/angular controls, upper/lower bounds
    std::vector<double> lb;
    lb.resize(oneDimVector.size(), controlLowerBound);
    std::vector<double> ub;
    ub.resize(oneDimVector.size(), controlUpperBound);
    optimizer.set_lower_bounds(lb);
    optimizer.set_upper_bounds(ub);

    for(Constraint& constraint : m_constraints)
    {
        constraint.setActualSystem(m_model, t0);
        optimizer.add_inequality_constraint(Constraint::wrapConstraintFunctionObject, &constraint);
    }
    ///the control constraints (up1, up2)
    for(ControlConstraint& constrolConstraint : m_controlConstraints)
    {
        constrolConstraint.setTime(t0);
        optimizer.add_inequality_constraint(ControlConstraint::wrapConstraintFunctionObject, &constrolConstraint);
    }

    double funcValue = 0.0;
    ///add a stop criterion to stop infinite optimization
    optimizer.set_ftol_rel(0.01);
    optimizer.set_xtol_rel(0.01);

    /// TCP-Client-Call

    if (m_socketClient && m_socketClient->connected()) {
        std::vector<std::vector<double> > controlServer = m_socketClient->sendRequest(optimizer, control, constraints, costFunction, x0, m_model);
        oneDimVector = VectorHelper::reshapeXdTo1d(controlServer);
    }
    else {
        nlopt::result result = optimizer.optimize(oneDimVector, funcValue);
    }

    ///convert result vector back
    std::cout << "Ende der Optimierung" << std::endl;
    return VectorHelper::reshapeXd(oneDimVector, 2);
}

void MPCController::initSocketClient() {
    //TOBIAS: uncomment leads to local optimisation on raspberry pi
    m_socketClient = std::make_shared<SocketClient>();
}
