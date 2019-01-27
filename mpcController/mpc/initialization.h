#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <QObject>
#include <chrono>
#include <thread>
#include <functional>
#include <vector>
#include <QThread>
#include "robot.h"
#include "worker.h"
#include <QCoreApplication>

/**
 * @brief The Initialization class, the upper-layer of robots, deal with the whole robot set and its initialization process and further main process of optimization
 */
class Initialization : public QObject
{
    Q_OBJECT

public:
    Initialization();
    Initialization(const std::vector<std::string>& robotsId, const std::vector<std::vector<double> >& starts,
                            const std::vector<std::vector<double> >& targets);
    ~Initialization();
    bool shutDownOpt(const std::vector<bool>& atTarget);

signals:
    void getRestSubscribed();
    void unsubscribeALL();
    void reSubscribed();
    void reachedTarget(const std::string &id);
    void controlsOut(const std::vector<double> &control, const std::string &id);
    void trajOut(const std::vector<std::vector<double> > &traj, const std::string &id);
    void operate(const std::vector<std::vector<double> > &controls, const Robot &robots, const double &t0);

public slots:
    void startOptimization(); ///main process
    void getIDs(const std::vector<std::string> Ids);
    void locationsIn(const std::map<std::string, std::vector<double> > &loca);
    void targetsIn(const std::map<std::string, std::vector<std::vector<double> > > &targ);
    void forceStopIn(const std::string &id);
    void handleResults(const std::vector<std::vector<double> > &result);
    void resume(const std::string& id);

private:
    ///initialize the entire set of robots when the class is first called
    std::map<std::string, std::vector<std::vector<double> > > InitialControl();
    std::map<std::string, std::vector<std::vector<double> > > RestartControl(
            const std::map<std::string, std::vector<std::vector<double> > > &controls, const std::string &Id);
    std::map<std::string, std::vector<Cell> > InitialCells();
    std::vector<Robot> InitialRobots();
    std::map<std::string, std::vector<Constraint> > InitialConstraints();
    std::vector<ControlConstraint> InitialControlConstraints();

    ///updating of single robot after it started to run to keep up with the location
    std::map<std::string, std::vector<Cell> >  UpdateSingleCarCells(const std::map<std::string, std::vector<Cell> >& ini_cells, const std::string &id);
    std::vector<Robot> UpdateSingleCarRobots(const std::vector<Robot> &ini_robots, const std::string &id);
    std::map<std::string, std::vector<Constraint> > UpdateSingleCarConstraints(const std::map<std::string, std::vector<Constraint> > &ini_constraint, const std::string &id);

    ///initialize a single robot when it has been forceStopped
    std::map<std::string, std::vector<std::vector<double> > > InitialStoppedControl(const std::map<std::string, std::vector<std::vector<double> > >  &controls);
    std::map<std::string, std::vector<Cell> > InitialStoppedCells(const std::map<std::string, std::vector<Cell> > &cells);
    std::vector<Robot> InitialStoppedRobots(const std::vector<Robot> &robots);
    std::map<std::string, std::vector<Constraint> > InitialStoppedConstraints(const std::map<std::string, std::vector<Constraint> > &constraints);

    ///the unique ID of the car, when publish the control, only one car should do it and listen to it
    std::string carUniqID = QString::fromUtf8(getenv("HOSTNAME")).toStdString();
    ///whether IDs, postion, targets are unsubscribed
    bool unsubscribedAll = false;
    ///total number of the robot cars
    size_t car_numbers;
    ///predict horizon
    size_t ini_N = horizon;
    ///the car needs to stop
    std::string m_stopId = "";
    ///vector of all robots' IDs
    std::vector<std::string> ini_ID;
    ///vector of all robots' starting points, for initializing the optimizing
    std::vector<std::vector<double> > ini_starts;
    ///vector of all robots' target points, for initializing the optimizing
    std::vector<std::vector<double> > ini_targets;
    ///hold all targets for every robots, in case the order will change
    std::map<std::string, std::vector<std::vector<double> > > m_targetsMap;
    ///hold start positions for every robots, in case the order will change
    std::map<std::string, std::vector<double> > m_locationsMap;
    ///whether the optimization from another thread has finished
    bool m_optFinished;
    ///get result from another thread
    std::vector<std::vector<double> > m_result;
    ///whether the startOptimization() method has started
    bool optStarted = false;
};

#endif // INITIALIZATION_H
