#ifndef GENERALTOPIC_H
#define GENERALTOPIC_H
#include <QObject>
#include <vector>
#include <typeinfo>
#include <QTimer>
#include <QDebug>
#include <QRegularExpression>
#include <QMutex>

#include "vectorhelper.h"
#include "socketmpcthread.h"
#include "initialization.h"

/**
 * @brief The GeneralTopic class, count robot numbers, get all IDs via qmqtt; then get the locations and (sub-)targets for each of the robot.
 * Mainly holds the communication between Monitorring App, Localization group, Constructor group.
 */
class GeneralTopic : public QObject
{
    Q_OBJECT

public:
    explicit GeneralTopic(QObject *parent = nullptr);
    std::vector<std::string> getAllIDs();
    bool getUnsubscribedBool();

signals:
    void sendIDs(const std::vector<std::string> &Ids);
    void locationsOut(const std::map<std::string, std::vector<double> > &pos);
    void targetsOut(const std::map<std::string, std::vector<std::vector<double> > > &targ);
    void forceStopOut(const std::string &id);
    void resume(const std::string& id);

public slots:
    void subscribePosition();
    void subscribeRest();
    void receivedMsg(const QMQTT::Message msg);
    void sendControl(const std::vector<double> &control, const std::string &id);
    void sendTrajectory(const std::vector<std::vector<double> > &traj, const std::string &id);
    void unsubscribeAll();
    void sendStop(const std::string &id);
    void finish();

private:
    ///pointer of SocketMPC class instance
    QPointer<SocketMpcThread> m_socketMpc;

    ///instance of Initialization class
    Initialization m_intialization;

    ///whether the robot has successfully unsubsribed all topics
    bool unsubscribedAll = false;

    ///contains all robots' IDs
    std::vector<std::string> m_robotsIDs;

    ///single car's start position
    std::vector<double> m_locationSingle;

    ///single car's targets
    std::vector<std::vector<double> > m_targetSingle;

    /// map of all robots IDs and their start positions, simple way than get one by one
    std::map<std::string, std::vector<double> > m_posAll;

    /// map of all robots IDs and their sub-targets, as monitoring group may only send this signal once, can't afford lose it
    std::map<std::string, std::vector<std::vector<double> > > m_targAll;

    /// which car is force to stop, as monitoring group may only send this signal once, need to catch the info
    std::string stopID;
    QMutex m_blockId;
};

#endif // GENERALTOPIC_H
