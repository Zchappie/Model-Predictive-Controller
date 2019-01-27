#include "initialization.h"
#include <time.h>
#include "controller.h"
#include <iostream>

Initialization::Initialization()
{}
/**
 * @brief Initialization::Initialization constructor, pass all the parameters into the class
 * @param robotsId, all robots' IDs
 * @param starts, all robots' start positions(x,y,theta)
 * @param targets, all robots' (sub-)targets
 */
Initialization::Initialization(const std::vector<std::string> &robotsId, const std::vector<std::vector<double> >& starts,
                               const std::vector<std::vector<double> >& targets):
    ini_N(horizon),
    m_optFinished(false)
{
    qDebug("[Initialization] Initializing the initialization class...");
    ini_ID = robotsId;
    ini_starts = starts;
    ini_targets = targets;
}

Initialization::~Initialization()
{
}

/**
 * @brief Initialization::getIDs, receive IDs from GeneralTopic class, then trigger the RobotsTopic class to get the locations/targets
 * @param Ids, all robots' IDs passed from GeneralTopic class
 */
void Initialization::getIDs(const std::vector<std::string> Ids)
{
    ini_ID = Ids;
    car_numbers = ini_ID.size();
    qDebug()<<"[Initialization] Car number is" <<car_numbers;
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        qDebug()<<"[Initialization] Get IDs from qmqtt"<<QString::fromStdString(ini_ID.at(i));
    }
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        emit getRestSubscribed();
    }
}

/**
 * @brief Initialization::locationsIn, get the location from generalTopic class
 * @param loca, map of ids and locations
 */
void Initialization::locationsIn(const std::map<std::string, std::vector<double> > &loca)
{
    m_locationsMap.clear();
    m_locationsMap = loca;
}

/**
 * @brief Initialization::targetsIn, as targets are set manually, it is supposed to arrive later than start position,
 * if all needed info is here(start position, targets, ids match each other), then unsubscribed;
 * otherwise, wait
 * @param targ, map of IDs and their (sub-)targets
 */
void Initialization::targetsIn(const std::map<std::string, std::vector<std::vector<double> > > &targ)
{
    //set stop id empty to restart
    m_stopId = "";
    m_targetsMap.clear();
    m_targetsMap = targ;
    if(!optStarted)
    {
        if(m_locationsMap.size()==m_targetsMap.size() && m_targetsMap.size()==ini_ID.size())
        {
            startOptimization();
        }
    }
}

/**
 * @brief Initialization::forceStopIn, pass in the stop sign when the instruction comes
 * @param id, which robot to stop
 */
void Initialization::forceStopIn(const std::string &id)
{
    m_stopId.clear();
    m_stopId = id;
    qDebug()<<"[Initialization] Force Stop is working....";
}

void Initialization::resume(const std::string& id) {
    if (m_stopId == id) {
        m_stopId.clear();
    }
}

/**
 * @brief Initialization::InitialControl, called when the controller first starts
 * @return an empty control set for all of the cars
 */
std::map<std::string, std::vector<std::vector<double> > > Initialization::InitialControl()
{
    std::vector<std::vector<double> > control;
    std::map<std::string, std::vector<std::vector<double> > > controls;
    for (size_t i = 0; i < ini_N; i++)
    {
        control.push_back({0.0, 0.0});
    }
    for (size_t i = 0; i < ini_ID.size(); i++)
    {
        controls[ini_ID.at(i)] = control;
    }
    return controls;
}

/**
 * @brief RestartControl, called when the single car re-starts after the controller starts
 * @param controls, the controls of all cars
 * @return an re-assigned control set
 */
std::map<std::string, std::vector<std::vector<double> > > Initialization::RestartControl(
        const std::map<std::string, std::vector<std::vector<double> > >& controls, const std::string& Id)
{
    std::map<std::string, std::vector<std::vector<double> > > newControls = controls;
    ///iterate over map (for all cars, with all vectors)
    ///cbegin is const, so everything in the cascade is then by definition const
    for(auto itCars = newControls.begin(); itCars != newControls.end(); itCars++)
    {
        ///find the car which needs to restart
        if (itCars->first == Id)
        {
            ///iterate now over all controls for this car, and re-assign 0s
            for(auto itCar = (*itCars).second.begin(); itCar != (*itCars).second.end(); itCar++)
            {
                for(auto it = (*itCar).begin(); it != (*itCar).end(); it++)
                {
                    *it = 0;
                }
            }
        }
    }
    return newControls;
}

/**
 * @brief Initialization::InitialCells, initialize the map of ID, blocked cells to pass to robot class
 * @return a map(IDs, blocked cells)
 */
std::map<std::string, std::vector<Cell> > Initialization::InitialCells()
{
    ///initialize the map
    std::map<std::string, std::vector<Cell> > cells;

    ///first, transform the double location to integer location
    std::vector<std::vector<size_t> > locations;
    std::vector<std::vector<double> > startVec = ini_starts;
    Cell cl;
    for(size_t i=0; i<startVec.size(); i++)
    {
        startVec.at(i).erase(startVec.at(i).begin()+2);
        ///mapC function only contains x,y(no angular)
        locations.push_back(cl.mapContinuousPositionToCells(startVec.at(i)));
    }

    ///second, expand the location to blocked cell sequences, and build/fill the map, total length: horizon+1
    for(size_t i=0; i<locations.size(); i++)
    {
        std::vector<Cell> locas;
        for(size_t j=0; j<(ini_N+1); j++)
        {
            Cell cell(locations.at(i).at(0), locations.at(i).at(1));
            locas.push_back(cell);
        }
        cells[ini_ID.at(i)] = locas;
    }
    return cells;
}

/**
 * @brief Initialization::InitialSolo, initialize the robot-object one by one
 * @return a vector of robot-object
 */
std::vector<Robot> Initialization::InitialRobots()
{
    std::vector<Robot> robots;
    for(size_t i=0; i<car_numbers; i++)
    {
        std::string iniID = ini_ID.at(i);
        std::vector<double> iniStart = ini_starts.at(i);
        std::vector<double> iniTarget = ini_targets.at(i);
        Robot rob(iniID, iniStart, iniTarget);
        robots.push_back(rob);
    }
    return robots;
}

/**
 * @brief Initialization::InitialConstraints, initialize the map of IDs and constraints
 * @return a map(IDs, constraint)
 */
std::map<std::string, std::vector<Constraint> > Initialization::InitialConstraints()
{
    ///initialize the map
    std::map<std::string, std::vector<Constraint> > newConstraints;

    std::vector<std::vector<double> > startVec = ini_starts;
    ///iterate over all cars
    for(size_t i=0; i<startVec.size(); i++)
    {
        startVec.at(i).erase(startVec.at(i).begin()+2);///map only contains x,y(no angular)
        std::vector<Constraint> constrainVec;
        ///every car every position is converted to a constraint
        std::vector<std::vector<double> > locations;
        double t = 0.0;
        for(size_t j=0; j<ini_N+1; j++)
        {
            locations.push_back(startVec.at(i));
            Constraint c(t, locations.at(j));
            constrainVec.push_back(c);
            t += sampleInterval;
        }
        newConstraints[ini_ID.at(i)] = constrainVec;
    }
    return newConstraints;
}

/**
 * @brief Initialization::InitialControlConstraints, initialize the control constraints
 * @return vector of control constraints
 */
std::vector<ControlConstraint> Initialization::InitialControlConstraints()
{
    std::vector<ControlConstraint> controlConstraints;

    double t = 0.0;
    ///every control time has to obey the control constraint
    for(size_t j=0; j<ini_N; j++)
    {
        ControlConstraint c(t);
        controlConstraints.push_back(c);
        t += sampleInterval;
    }
    return controlConstraints;
}

/**
 * @brief Initialization::UpdateSingleCarCells, re-initialize the cell for single car when it runs, to keep updating the location
 * @param ini_cells, all robots' blocked cells
 * @param id, id of the robot who needs to update
 * @return new map of all robots' blocked cells
 */
std::map<std::string, std::vector<Cell> > Initialization::UpdateSingleCarCells(const std::map<std::string, std::vector<Cell> >& ini_cells, const std::string &id)
{
    std::map<std::string, std::vector<Cell> > cells = ini_cells;
    std::map<std::string, std::vector<double> > locas = m_locationsMap;

    ///first, transform the double location to integer location
    std::vector<size_t> locations;
    std::vector<double> startVec = locas[id];
    Cell cl;
    startVec.erase(startVec.begin()+2); ///delete the angle
    locations = cl.mapContinuousPositionToCells(startVec);

    ///second, expand the location to blocked cell sequences, and build/fill the map, total length: horizon+1
    std::vector<Cell> loca;
    for(size_t j=0; j<(ini_N+1); j++)
    {
        Cell cell(locations.at(0), locations.at(1));
        loca.push_back(cell);
    }
    cells[id] = loca;
    return cells;
}

/**
 * @brief Initialization::UpdateSingleCarRobots, re-initialize the robot for single car when it runs, to keep updating the location
 * @param ini_robots, all robots' instances
 * @param id, id of the robot who needs to update
 * @return new map of all robots
 */
std::vector<Robot> Initialization::UpdateSingleCarRobots(const std::vector<Robot> &ini_robots, const std::string &id)
{
    std::vector<Robot> robots = ini_robots;
    for(size_t i=0; i<robots.size(); i++)
    {
        if(robots.at(i).ID() == id)
        {
            Robot rob(id, m_locationsMap[id], m_targetsMap[id].at(0));
            robots.at(i) = rob;
            break;
        }
    }
    return robots;
}

/**
 * @brief Initialization::UpdateSingleCarConstraints, re-initialize the robot for single car when it runs, to keep updating the location
 * @param ini_constraint, all robots' constraints
 * @param id, id of the robot who needs to update
 * @return new map of all robot's constraints
 */
std::map<std::string, std::vector<Constraint> > Initialization::UpdateSingleCarConstraints(const std::map<std::string, std::vector<Constraint> > &ini_constraint, const std::string &id)
{
    std::map<std::string, std::vector<Constraint> > newConstraints = ini_constraint;
    std::map<std::string, std::vector<double> > locas = m_locationsMap;
    std::vector<double> startVec = locas[id];
    startVec.erase(startVec.begin()+2);///map only contains x,y(no angular)
    std::vector<Constraint> constrainVec;
    double t = newConstraints[id].at(0).constraintTime();
    for(size_t j=0; j<ini_N+1; j++)
    {
        Constraint c(t, startVec);
        constrainVec.push_back(c);
        t += sampleInterval;
    }
    newConstraints[id] = constrainVec;
    return newConstraints;
}

/**
 * @brief Initialization::InitialStoppedControl, add the new robot's control at the end of control map
 * @param controls, the map contains all running robots' controls
 * @return new controls which also include the newest car (probably forceStopped before)
 */
std::map<std::string, std::vector<std::vector<double> > > Initialization::InitialStoppedControl(const std::map<std::string, std::vector<std::vector<double> > > &controls)
{
    std::vector<std::vector<double> > control;
    std::map<std::string, std::vector<std::vector<double> > >  newControls = controls;
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        if(controls.find(ini_ID.at(i)) == controls.end())
        {
            for (size_t i = 0; i < ini_N; i++)
            {
                control.push_back({0.0, 0.0});
            }
            newControls[ini_ID.at(i)] = control;
        }
    }
    return newControls;
}

/**
 * @brief Initialization::InitialStoppedCells, add the new robot's blocked cells into the cell map
 * @param cells, the map contains all running robots' cells
 * @return new cell map which also include the newest car (probably forceStopped before)
 */
std::map<std::string, std::vector<Cell> > Initialization::InitialStoppedCells(const std::map<std::string, std::vector<Cell> > &cells)
{
    Cell cl;
    std::map<std::string, std::vector<Cell> > newCells = cells;
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        if(cells.find(ini_ID.at(i)) == cells.end())
        {
            ///if not found the robot, then it's new
            std::vector<size_t> location;
            std::vector<double> locaVec = m_locationsMap[ini_ID.at(i)];
            locaVec.erase(locaVec.begin()+2);
            location = cl.mapContinuousPositionToCells(locaVec);

            std::vector<Cell> locas;
            for(size_t i = 0; i<ini_N+1; i++)
            {
                Cell cell(location.at(0), location.at(1));
                locas.push_back(cell);
            }
             newCells[ini_ID.at(i)] = locas;
        }
    }
    return newCells;
}


/**
 * @brief Initialization::InitialStoppedRobots, add the new robot into the robots vector
 * @param robots, the vector contains all running robots
 * @return new robots vector which also include the newest car (probably forceStopped before)
 */
std::vector<Robot> Initialization::InitialStoppedRobots(const std::vector<Robot> &robots)
{
    std::vector<Robot> newRobot = robots;
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        bool notFound = true;
        for(size_t j=0; j<robots.size(); j++)
        {
            if(robots.at(j).ID() == ini_ID.at(i))
            {
                ///found, then do nothing
                notFound = false;
                break;
            }
        }
        if(notFound)
        {
            Robot rob(ini_ID.at(i), m_locationsMap[ini_ID.at(i)], m_targetsMap[ini_ID.at(i)].at(0));
            newRobot.push_back(rob);
        }
    }
    return newRobot;
}

/**
 * @brief Initialization::InitialStoppedConstraints, add the new robot's constraints into the constraint map
 * @param constraints, the map contains all running robots
 * @return new constraints map which also include the newest car (probably forceStopped before)
 */
std::map<std::string, std::vector<Constraint> > Initialization::InitialStoppedConstraints(const std::map<std::string, std::vector<Constraint> > &constraints)
{
    std::map<std::string, std::vector<Constraint> > newConstraints = constraints;
    for(size_t i=0; i<ini_ID.size(); i++)
    {
        if(constraints.find(ini_ID.at(i)) == constraints.end())
        {
            ///not found
            std::vector<double> startVec = m_locationsMap[ini_ID.at(i)];
            startVec.erase(startVec.begin()+2);
            auto it = newConstraints.begin();
            double t = it->second.at(0).constraintTime();
            std::vector<std::vector<double> > locations;
            std::vector<Constraint> constrainVec;
            for(size_t j=0; j<ini_N+1; j++)
            {
                locations.push_back(startVec);
                Constraint c(t, locations.at(j));
                constrainVec.push_back(c);
                t += sampleInterval;
            }
            newConstraints[ini_ID.at(i)] = constrainVec;
        }
    }
    return newConstraints;
}

/**
 * @brief Initialization::shutDownOpt, shut down the control when all the cars are at their targets
 * @param atTarget, vector of boolean of whether cars are at the targets
 * @return bool value, when True shut down, when false continue the process
 */
bool Initialization::shutDownOpt(const std::vector<bool> &atTarget)
{
    bool shutDown = true;
    for(size_t i=0; i<atTarget.size(); i++)
    {
        shutDown = shutDown * atTarget.at(i);
    }
    return shutDown;
}

/**
 * @brief Initialization::startOptimisation, hold the main process of optimization when all info is here and unsubscribed the postion/targets firstly;
 * call MPCController class to optimize the controls, and publish the optimized controls to the robot one by one, step by step;
 * when all the robots reach their all targets, subscribe the position/targets again to wait for the next instruction.
 */
void Initialization::startOptimization()
{
    std::cout << "started Optimization" << std::endl;
    optStarted = true;
    double t0 = 0.0;
    std::vector<std::string> ids = ini_ID;
    std::map<std::string, std::vector<std::vector<double> > > ends = m_targetsMap;
    ini_targets.clear();
    for(auto i=ends.begin(); i!=ends.end(); i++)
    {
        ini_targets.push_back(i->second.at(0));
    }
    std::map<std::string, std::vector<double> > starts = m_locationsMap;
    ini_starts.clear();
    for(auto i=starts.begin(); i!=starts.end(); i++)
    {
        ini_starts.push_back(i->second);
    }

    ///initialize all the components for optimization
    std::vector<Robot> robots = InitialRobots();
    std::map<std::string, std::vector<Cell> > cells;
    cells = InitialCells();
    std::map<std::string, std::vector<std::vector<double> > > controls;
    controls = InitialControl();
    std::map<std::string, std::vector<Constraint> > constraints;
    constraints = InitialConstraints();
    std::vector<ControlConstraint> controlConstraint;
    controlConstraint = InitialControlConstraints();

    ///Loop that robots are running until target is reached or forcely stop comes
    while (!robots.empty())
    {
        ///start to optimize
        for (size_t i = 0; i < robots.size(); i++)
        {
            robots.at(i).setConstraints(constraints);
            robots.at(i).setControlConstraints(controlConstraint);
            robots.at(i).receiveCells(cells);
        }
        for (size_t i = 0; i < robots.size(); i++)
        {
            //if it was force-stopped, do nothing
            if (robots.at(i).ID() != m_stopId) {
                m_optFinished = false;
                Controller controller(robots.at(i).controller(), controls[robots.at(i).ID()], robots.at(i).getConstraints(constraints),
                                      controlConstraint, robots.at(i).currentPosition(), robots.at(i).target(), t0);
                QObject::connect(&controller, SIGNAL(finished(const std::vector<std::vector<double> > &)), this, SLOT(handleResults(const std::vector<std::vector<double> > &)));
                controller.start();
                while (!m_optFinished)
                {
                    QCoreApplication::processEvents(QEventLoop::AllEvents);
                    QThread::msleep(50);
                }
                controls[robots.at(i).ID()] = m_result;
                std::map<std::string, std::vector<Cell> > newCells = robots.at(i).sendCells(controls[robots.at(i).ID()]);
                for (size_t j = 0; j < robots.size(); j++)
                {
                    if(i != j){robots.at(j).receiveCells(newCells);}
                }
                for(size_t k=0; k<robots.at(i).m_trajectory.size();k++)
                {
                    qDebug()<<"[Main] The"<<i<<"robot's"<<k<<"trajectory is"<<robots.at(i).m_trajectory.at(k);
                }
                QCoreApplication::processEvents();
                ///send the controls
                if (carUniqID == robots.at(i).ID() || carUniqID == "")
                {
                    emit controlsOut(controls[robots.at(i).ID()].at(0), robots.at(i).ID());

                    emit trajOut(robots.at(i).m_trajectory, robots.at(i).ID());
                }
            }//--is not stopped
        }
        ///shift the control
        for (size_t i = 0; i < robots.size(); i++)
        {
            //if it was force-stopped, do nothing
            if (robots.at(i).ID() != m_stopId) {
                std::vector<std::vector<double> > u;
                u = robots.at(i).shiftTime(controls[robots.at(i).ID()]);
                controls[robots.at(i).ID()].clear();
                controls[robots.at(i).ID()] = u;
                robots.at(i).setCurrentPos(robots.at(i).m_trajectory.at(1));
            }
        }
        QCoreApplication::processEvents();
        ///************************************
        /// time break for half second
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); //can use sleep_until
        ///************************************
        auto itRobot = robots.begin();
        while (itRobot != robots.end())
        {
            ///remove the car who has been forcely stopped, and wait for the new start
            if((*itRobot).ID() == m_stopId)
            {
                qDebug()<<"[Initialization] Force Stop is working....";
                itRobot = robots.erase(itRobot);
            }
            else
            {
                ///remove the car who has reached the final target
                if ((*itRobot).hasReachedTarget())
                {
                    emit reachedTarget((*itRobot).ID());
                    ///first, delete reached target
                    std::vector<std::vector<double> > vec = ends[(*itRobot).ID()];
                    vec.erase(vec.begin());
                    ends[(*itRobot).ID()] = vec;
                    ///if there is a (sub-)target infront, then set as new ends
                    if(!vec.empty())
                    {
                        ///re-set this robot
                        Robot rob((*itRobot).ID(), (*itRobot).currentPosition(), vec.at(0));
                        (*itRobot) = rob;
                    }
                    else
                    {
                        ///before erase the robot, need to set a const constraint/cells for others
                        size_t lenCell = cells[(*itRobot).ID()].size();
                        cells[(*itRobot).ID()].clear();
                        cells[(*itRobot).ID()] = (*itRobot).setFinalCell(lenCell);
                        size_t lenConstraint = constraints[(*itRobot).ID()].size();
                        double constraintTime = constraints[(*itRobot).ID()].at(0).constraintTime();
                        constraints[(*itRobot).ID()].clear();
                        constraints[(*itRobot).ID()] = (*itRobot).setFinalConstraint(lenConstraint, constraintTime);

                        itRobot = robots.erase(itRobot);
                    }
                }
                itRobot++;
            }

        }
        ///update the location from localization group
        for (size_t i = 0; i < robots.size(); i++)
        {
            cells = UpdateSingleCarCells(cells, robots.at(i).ID());
            constraints = UpdateSingleCarConstraints(constraints, robots.at(i).ID());
            robots = UpdateSingleCarRobots(robots, robots.at(i).ID());
        }
        ///to start the forceStopped car to start again
        controls = InitialStoppedControl(controls);
        robots = InitialStoppedRobots(robots);
        cells = InitialStoppedCells(cells);
        constraints = InitialStoppedConstraints(constraints);
    }
    emit(reSubscribed());
}

void Initialization::handleResults(const std::vector<std::vector<double> > &result)
{
    m_optFinished = true;
    m_result = result;
}
