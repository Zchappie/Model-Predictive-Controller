#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include "constraint.h"
#include "controlconstraint.h"
#include "model.h"
#include "socketclient.h"
#include "vectorhelper.h"
#include "nlopt.hpp"

#include "Tests/testobstacle.h"

#include <QtCore/QString>
#include <QDebug>

/**
 * @brief The MPCController class represents an MPC controller for each robot, regarding as the formulation of the OCP, therefore including the cost function, the constraints,
 * and the dynamics (kinematic model, is here presented as member)
 */
class MPCController
{
public:
    MPCController(const std::string& car);
    std::vector<std::vector<double> > optimize(const std::vector<std::vector<double> >& control, const std::vector<Constraint>& constraints,
                                               const std::vector<ControlConstraint> &controlConstraints, const std::vector<double>& x0,
                                               const std::vector<double>& target, const double& t0);
    void addConstraints(const std::vector<Constraint>& constraints);
    void addControlConstraint(const std::vector<ControlConstraint>& controlConstraint);
    void initSocketClient();
private:
    ///ID of the car
    std::string m_car;
    ///horizon length
    size_t m_N;
    ///sampling step size
    double m_T;
    ///fraction for control value
    double m_lambda;
    ///constraints
    std::vector<Constraint> m_constraints;
    ///control constraints
    std::vector<ControlConstraint> m_controlConstraints;
    ///underlying model
    std::shared_ptr<Model> m_model;
    ///test purpose
    std::vector<TestObstacle> m_testObstacle;
    ///client to perform the optimisation on an external server
    std::shared_ptr<SocketClient> m_socketClient;
};

#endif // MPCCONTROLLER_H
