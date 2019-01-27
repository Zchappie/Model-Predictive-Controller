#include "robot.h"
#include "cell.h"

/**
 * @brief Robot::Robot constructs the car with the current parameters
 * @param robotId, id of robot
 * @param start, start position
 * @param target, target position
 * @param N, prediction horizon length
 * @param T, sampling delta
 * @param lambda
 */
Robot::Robot(const std::string& robotId, const std::vector<double>& start, const std::vector<double>& target) :
    id(robotId),
    m_currentPos(start),
    m_target(target)
{
    qDebug("[Robot] Initializing the robot class...");
    m_N = horizon;
    m_T = sampleInterval;
    m_lambda = lambda;
    m_minDistance = minDistance;
    m_controller = std::make_shared<MPCController>(robotId);
}

/**
 * @brief Robot::optimize calls the MPC-Controller to optimize the controls
 * @param control, initialized control vector
 * @param x0, current position
 * @param target
 * @param t0
 * @return optimized control
 */
std::vector<std::vector<double> > Robot::optimize(const std::vector<std::vector<double> >& control, const std::vector<double>& x0,
                                                  const std::vector<double>& target, const double &t0)
{
    qDebug("[Robot] Running optimize method...");
    std::vector<Constraint> constraints = getConstraints(m_constraints);
    ///only use constraints, which are from other robots, not the own!

    std::vector<ControlConstraint> controlCconstraints = m_controlConstraint;
    std::vector<std::vector<double> >controls = m_controller->optimize(control, constraints, controlCconstraints, x0, target, t0);
    #if QT_VERSION >= QT_VERSION_CHECK(5,10,0)
        qDebug()<<"[Robot] Controls are" <<controls;
    #endif
    return controls;
}

/**
 * @brief Robot::setConstraints, get the entire constraint map of whole car set
 * @param constraints, map(IDs, constraints) save to local class
 */
void Robot::setConstraints(const std::map<std::string, std::vector<Constraint> >& constraints)
{
    m_constraints = constraints;
}

/**
 * @brief Robot::setControlConstraints, get the control constraint of the car
 * @param controlConstraints
 */
void Robot::setControlConstraints(const std::vector<ControlConstraint> &controlConstraints)
{
    m_controlConstraint = controlConstraints;
}

/**
 * @brief Robot::currentPosition
 * @return
 */
std::vector<double> Robot::currentPosition() const
{
    return m_currentPos;
}

std::vector<double> Robot::target() const
{
    return m_target;
}

std::string Robot::ID() const
{
    return id;
}

void Robot::setCurrentPos(const std::vector<double>& pos)
{
    m_currentPos = pos;
}

/**
 * @brief Robot::shiftTime, remove first value in control, append 0 at the end
 * @param control, the control sequence
 * @return the new control sequence
 */
std::vector<std::vector<double> > Robot::shiftTime(const std::vector<std::vector<double> > &control)
{
    ///remove first value in control, append 0 at the end
    std::vector<std::vector<double> > u = control;
    u.erase(u.begin());
    u.push_back({0.0, 0.0});
    m_t0 = m_t0 + m_T;
    return u;
}

/**
 * @brief Robot::hasReachedTarget, whether the car has reached the target
 * @return boolean value, true=reached, false=not reach
 */
bool Robot::hasReachedTarget() const
{
    std::vector<double> curPos = m_currentPos;
    if (curPos.size() > 2) {
        curPos.erase(curPos.begin()+2);
    }
    std::vector<double> targ = m_target;
    if (targ.size() > 2) {
        targ.erase(targ.begin()+2);
    }
    if (VectorHelper::norm2(VectorHelper::sub(curPos, targ)) < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Robot::setFinalConstraint, together with "hasReachedTarget" or "forceStop" method, after this car finished/stop, set a const cells for others
 * @param vecLenth, the length of the constraint
 * @return blocked cell(this car's current state)
 */
std::vector<Cell> Robot::setFinalCell(const size_t vecLen) const
{
    std::vector<Cell> cls;
    std::vector<std::vector<size_t> > clsIn;
    Cell cl;
    std::vector<double> state = m_currentPos;
    state.erase(state.begin()+2);
    for(size_t i=0; i<vecLen; i++)
    {
        clsIn.push_back(cl.mapContinuousPositionToCells(state));
        Cell cell(clsIn.at(i).at(0), clsIn.at(i).at(1));
        cls.push_back(cell);
    }
    return cls;
}

/**
 * @brief Robot::setFinalConstraint, together with "hasReachedTarget" method, after this car finished, set a const constraint for others
 * @param vecLen, the length of the constraint
 * @param t, the start time of the constraint
 * @return const constraint formed by its target
 */
std::vector<Constraint> Robot::setFinalConstraint(const size_t vecLen, const double t) const
{
    std::vector<Constraint> constraint;
    std::vector<double> targ = m_target;
    targ.erase(targ.begin()+2);
    double t0 = t;
    for(size_t i=0; i<vecLen; i++)
    {
        Constraint c(t0, targ);
        constraint.push_back(c);
        t0 = t0 + m_T;
    }
    return constraint;
}

/**
 * @brief Robot::sendCells, after optimization of the referred car, send the new map(1D)
 * @param u, control sequence of the referred car
 * @return new, optimized map (1D map)
 */
std::map<std::string, std::vector<Cell> > Robot::sendCells(const std::vector<std::vector<double> >& u)
{
    qDebug("[Robot] Sending cells...");
    std::map<std::string, std::vector<Cell> > cell;

    ///calculate the trajectory given controls
    std::vector<std::vector<double> > traj;
    traj.push_back(m_currentPos);
    Model car;
    for(size_t i=0; i<u.size(); i++)
    {
        traj.push_back(car.calcNext(traj.back(), u.at(i)));
    }

    ///save the current trajectory for further use
    m_trajectory.clear();
    m_trajectory = traj;

    ///then erase the angular pose, convert to blocked cells
    std::vector<std::vector<size_t> > trajCell;
    std::vector<Cell> cellVec;
    Cell cl;
    for(size_t i=0; i<traj.size(); i++)
    {
        traj.at(i).erase(traj.at(i).begin()+2);
        trajCell.push_back(cl.mapContinuousPositionToCells(traj.at(i))); ///total length: horizon+1
        Cell c(trajCell.at(i).at(0), trajCell.at(i).at(1));
        cellVec.push_back(c);
    }
    cell[id] = cellVec;
    return cell;
}

/**
 * @brief Robot::receiveCells, receive the cells from other cars and converts them directly to constraints
 * @param cells, map of sent-map (1D map)
 */
void Robot::receiveCells(const std::map<std::string, std::vector<Cell> >& cells)
{
    qDebug("[Robot] Receiving cells...");
    ///convert this here directly to constraints
    ///iterate over map (all cars, with all vectors)
    for(auto itCells = cells.cbegin(); itCells != cells.cend(); itCells++)
    {
        ///iterate now over all received cells for this car
        for(auto itCell = (*itCells).second.begin(); itCell != (*itCells).second.end(); itCell++)
        {
            std::vector<double> continuousPos;
            continuousPos.push_back((*itCell).x());
            continuousPos.push_back((*itCell).y());

            bool foundConstraint = false;
            ///iterate the constraints inside the same car
            for(auto it = m_constraints.at(itCells->first).begin(); it != m_constraints.at(itCells->first).end(); it++)
            {
                ///if a constraint with same time instant already exists, then replace
                if((*it).constraintTime() == (*itCell).time())
                {
                    Constraint constraint ((*itCell).time(), continuousPos);
                    (*it) = constraint;
                    foundConstraint = true;
                    break;
                }
            }
            if(!foundConstraint)
            {
                ///if a constraint with same time instant doesnt exist, then append at the end
                Constraint constraint ((*itCell).time(), continuousPos);
                m_constraints.at(itCells->first).push_back(constraint);
            }
        }
    }
}

/**
 * @brief Robot::controller
 * @return
 */
std::shared_ptr<MPCController> Robot::controller() const {
   return m_controller;
}

std::vector<Constraint> Robot::getConstraints(const std::map<std::string, std::vector<Constraint> >& allConstraints) const {
    std::vector<Constraint> constraints;
    for (auto itConstraintVec = allConstraints.begin(); itConstraintVec != allConstraints.end(); itConstraintVec++)
    {
        if (itConstraintVec->first != id)
        {
            for (auto itConstraint = itConstraintVec->second.begin(); itConstraint != itConstraintVec->second.end(); itConstraint++)
            {
                constraints.push_back((*itConstraint));
            }
        }
    }
    return constraints;
}
