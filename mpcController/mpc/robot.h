#ifndef ROBOT_H
#define ROBOT_H

#include "mpccontroller.h"
#include "constraint.h"

#include "Tests/testobstacle.h"

#include <QDebug>

#include <vector>
#include <memory>
#include <map>
#include <string>


/**
 * @brief The Robot class models and administrates the underlying models, constraints
 * Each car handles its own communication, constraints, time shift, and mpc-controller
 */
class Robot
{
public:
    Robot(const std::string& robotId, const std::vector<double>& start, const std::vector<double>& target);
    std::vector<std::vector<double> > optimize(const std::vector<std::vector<double> >& control, const std::vector<double>& x0, const std::vector<double>& target, const double &t0);
    void applyControl(const std::vector<double>& control);
    std::vector<std::vector<double> > shiftTime(const std::vector<std::vector<double> >& control);
    std::vector<double> currentPosition() const;
    std::vector<double> target() const;
    std::string ID() const;
    void setConstraints(const std::map<std::string, std::vector<Constraint> >& constraints);
    void setControlConstraints(const std::vector<ControlConstraint>& controlConstraints);
    void setCurrentPos(const std::vector<double>& pos);

    void receiveCells(const std::map<std::string, std::vector<Cell> > &cells);
    std::map<std::string, std::vector<Cell> > sendCells(const std::vector<std::vector<double> > &u);

    bool hasReachedTarget() const;
    bool forceStop() const;
    std::vector<Cell> setFinalCell(const size_t vecLen) const;
    std::vector<Constraint> setFinalConstraint(const size_t vecLen, const double t) const;


    ///current predicted trajectory(x, y, theta)
    //TOBIAS: could we please put variables in private?
    std::vector<std::vector<double> > m_trajectory;

    std::shared_ptr<MPCController> controller() const;
    std::vector<Constraint> getConstraints(const std::map<std::string, std::vector<Constraint> >& allConstraints) const;

private:
    std::string id;
    ///current time
    double m_t0;
    ///MPC-controller to calculate the next
    std::shared_ptr<MPCController> m_controller;
    ///current position
    std::vector<double> m_currentPos;
    ///target
    std::vector<double> m_target;
    ///horizon length
    size_t m_N;
    ///sampling step size
    double m_T;
    ///fraction for control value
    double m_lambda;
    ///minimal safe distance
    double m_minDistance;
    ///constraints (received from the other cars)
    std::map<std::string, std::vector<Constraint> > m_constraints;
    ///controlConstraint
    std::vector<ControlConstraint> m_controlConstraint;
};

#endif // CAR_H
