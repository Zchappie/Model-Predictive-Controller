#include "generaltopic.h"

#include <QtCore/QMetaType>

/**
 * @brief GeneralTopic::GeneralTopic, constructor of class, start all connection
 * @param parent
 */

GeneralTopic::GeneralTopic(QObject *parent):
    QObject (parent)
{
    qRegisterMetaType<std::vector<std::vector<double> >>("std::vector<std::vector<double> >");
    m_socketMpc = new SocketMpcThread();
    ///build subscribe for IDs
    QObject::connect(m_socketMpc,SIGNAL(connected()),this,SLOT(subscribePosition()));
    ///build subscribe for rests
    QObject::connect(m_socketMpc,SIGNAL(connected()),this,SLOT(subscribeRest()));
    ///nuild parse the message of IDs
    QObject::connect(m_socketMpc,SIGNAL(received(QMQTT::Message)),this,SLOT(receivedMsg(QMQTT::Message)));
    QObject::connect(this, SIGNAL(sendIDs(std::vector<std::string>)), &m_intialization, SLOT(getIDs(std::vector<std::string>)));

    ///conncet RobotsTopic class to start subscribe position/targets and fetch them
    QObject::connect(this, SIGNAL(locationsOut(std::map<std::string, std::vector<double> >)), &m_intialization, SLOT(locationsIn(std::map<std::string, std::vector<double> >)));
    QObject::connect(this, SIGNAL(targetsOut(std::map<std::string, std::vector<std::vector<double> > >)), &m_intialization, SLOT(targetsIn(std::map<std::string, std::vector<std::vector<double> > >)));
    QObject::connect(this, SIGNAL(forceStopOut(std::string)), &m_intialization, SLOT(forceStopIn(std::string)));
    QObject::connect(this, SIGNAL(resume(std::string)), &m_intialization, SLOT(resume(std::string)));

    QObject::connect(&m_intialization, SIGNAL(unsubscribeALL()), this, SLOT(unsubscribeAll()));
    QObject::connect(&m_intialization, SIGNAL(getRestSubscribed()), this, SLOT(subscribeRest()));
    QObject::connect(&m_intialization, SIGNAL(controlsOut(std::vector<double>, std::string)), this, SLOT(sendControl(std::vector<double>, std::string)));
    QObject::connect(&m_intialization, SIGNAL(trajOut(std::vector<std::vector<double> >, std::string)), this, SLOT(sendTrajectory(std::vector<std::vector<double> >, std::string)));
    QObject::connect(&m_intialization, SIGNAL(reachedTarget(std::string)), this, SLOT(sendStop(std::string)));
    ///when all the robots reached their targets, re-subscribe all
    QObject::connect(&m_intialization, SIGNAL(reSubscribed()), this, SLOT(subscribePosition()));
    QObject::connect(&m_intialization, SIGNAL(reSubscribed()), this, SLOT(subscribeRest()));
    QObject::connect(m_socketMpc, SIGNAL(finished()), this, SLOT(finish()));
    ///start the socketMPC thread
    m_socketMpc->start();
}

/**
 * @brief GeneralTopic::subscribePosition, subscribe all topics under level "/robots/all/"
 */
void GeneralTopic::subscribePosition()
{
    m_socketMpc->subscribeTopic("/robots/all/#",1);
    qDebug() << "[GeneralTopic] Subscribed IDs...";
}

/**
 * @brief GeneralTopic::subscribeRest, after all IDs are gotten, then subscribe the rest topics, namely position/targets/forcestop
 */
void GeneralTopic::subscribeRest()
{
    m_socketMpc->subscribeTopic("/robots/+/position", 2);
    m_socketMpc->subscribeTopic("/robots/+/targets", 1);
    m_socketMpc->subscribeTopic("/robots/+/forceStop", 1);
    qDebug() << "[GeneralTopic] Subscribed rest of all...";
}

/**
 * @brief GeneralTopic::receivedMsg, parse all the got message from qmqtt
 * @param msg, the received message
 */
void GeneralTopic::receivedMsg(const QMQTT::Message msg)
{

    QString topicName = msg.topic();
    qDebug() << "[GeneralTopic] payload"<<msg.payload();
    if(topicName.contains("position"))
    {
        /// first clean the topic to figure out which robot
        QString tpc = msg.topic();
        QRegExp rt("/"); /// match /
        QStringList list0 = tpc.split(rt, QString::SkipEmptyParts);
        std::string IdOfPos = list0.at(1).toStdString();
        ///whether the received robot's ID is in m_robotsIDs vector
        m_blockId.lock();
        qDebug() << typeid(this).name() << ": locked" <<endl;
        for(size_t i=0; i<m_robotsIDs.size(); i++)
        {
            if(IdOfPos == m_robotsIDs.at(i))
            {
                /// then parse the message
                std::vector<double> m_position;
                QString position;
                position =msg.payload();
                QRegExp rx("[(),;]"); /// match () , ;
                QStringList list = position.split(rx, QString::SkipEmptyParts);
                for(auto i=0; i<list.size(); i++)
                {
                    m_position.push_back(list.at(i).toDouble());
                }
                m_position.at(2) = m_position.at(2) + M_PI_2;
                m_posAll[IdOfPos] = m_position;
                qDebug() << "[GeneralTopic] Parsed position are"<<m_position;
                emit locationsOut(m_posAll);
            }
        }
        m_blockId.unlock();
        qDebug() << typeid(this).name() << ": unlocked" <<endl;
    }
    else if(topicName.contains("targets"))
    {
        /// first clean the topic to figure out which robot
        QString tpc = msg.topic();
        QRegExp rt("/"); /// match /
        qDebug("->>>>>>>>>>>>>>>>>>>>>>>>>_--------------------->>>");
        QStringList list0 = tpc.split(rt, QString::SkipEmptyParts);
        std::string IdOfTargets = list0.at(1).toStdString();
        m_blockId.lock();
        qDebug() << typeid(this).name() << ": locked" <<endl;
        for(size_t i=0; i<m_robotsIDs.size(); i++)
        {
            if(IdOfTargets == m_robotsIDs.at(i))
            {
                /// then parse the message, as it may contain more than one target
                std::vector<std::vector<double> > m_target;
                QString target;
                target =msg.payload();
                QRegExp rx("[(),;]"); /// match () , ;
                QStringList list1 = target.split(rx, QString::SkipEmptyParts);
                std::vector<double> targetOneD;
                for(auto i=0; i < list1.size(); i++)
                {
                    targetOneD.push_back(list1.at(i).toDouble());
                }
                m_target = VectorHelper::reshapeXd(targetOneD, 2);
                for(size_t i=0; i<m_target.size(); i++)
                {
                    m_target.at(i).push_back(0.0);
                }
                /// then build the map of targets
                m_targAll[IdOfTargets] = m_target;
                qDebug() << "[GeneralTopic] Parsed targets are"<<m_target;
                m_blockId.unlock();
                qDebug() << typeid(this).name() << ": unlocked" <<endl;
                emit targetsOut(m_targAll);
            }
        }
    }
    else if (topicName.contains("resume")) {
        /// first clean the topic to figure out which robot
        QString tpc = msg.topic();
        QRegExp rt("/"); /// match /
        QStringList list0 = tpc.split(rt, QString::SkipEmptyParts);
        std::string IdOfPos = list0.at(1).toStdString();
        qDebug() << typeid (this).name() << " resumes" << endl;
        emit resume(IdOfPos);
    }
    else if(topicName.contains("forceStop"))
    {
        /// clean the topic to figure out which robot
        QString tpc = msg.topic();
        QRegExp rt("/"); /// match /
        QStringList list0 = tpc.split(rt, QString::SkipEmptyParts);
        std::string IdOfPoorGuy = list0.at(1).toStdString();
        qDebug() << "[GeneralTopic] ForceStop stops the poor guy"<<list0.at(1);
        emit forceStopOut(IdOfPoorGuy);
    }
    else if(topicName.contains("all"))
    {
        std::string id = msg.payload().toStdString();
        ///iterate over the id vector, if the id already exists, then pass, otherwise add at the end
        if(id != "")
        {
            bool duplicate = false;
            m_blockId.lock();
            qDebug() << typeid(this).name() << ": locked" <<endl;
            for(size_t i=0; i<m_robotsIDs.size(); i++)
            {
                if(id == m_robotsIDs.at(i))
                {
                    duplicate = true;
                    break;
                }
            }
            if(duplicate==false)
            {
                m_robotsIDs.push_back(id);
            }
            m_blockId.unlock();
            qDebug() << typeid(this).name() << ": unlocked" <<endl;
            if(!m_robotsIDs.empty())
            {
                emit sendIDs(m_robotsIDs);
            }
            for (auto itRobotIds = m_robotsIDs.begin(); itRobotIds != m_robotsIDs.end(); itRobotIds++)
            {
                qDebug() << "[GeneralTopic]" << "All robot IDs are" << QString::fromStdString(*itRobotIds);
            }
        }
    }
}

/**
 * @brief GeneralTopic::sendControl, send the control to topic "/robots/m_robotID/control"
 * @param control, vector of linear and angular velocity
 * @param id, id for which robot should listen
 */
void GeneralTopic::sendControl(const std::vector<double> &control, const std::string &id)
{
    QString u = "(";
    u = u + QString::number(control.at(0));
    u = u + ",";
    u = u + QString::number(control.at(1));
    u = u + ")";
    QString Id = QString::fromStdString(id);
    m_socketMpc->publishTopic("/robots/" + Id + "/control", u);
}

/**
 * @brief GeneralTopic::sendTrajectory, send the predicted trajectory to monitoring group to show on the monitorring app
 * @param traj, the trajectory needs to be sent
 * @param id, id of the robot who own this prediction
 */
void GeneralTopic::sendTrajectory(const std::vector<std::vector<double> > &traj, const std::string &id)
{
    QString tra;
    for(size_t i=0; i<traj.size(); i++)
    {
        tra = tra + "(";
        tra = tra + QString::number(traj.at(i).at(0));
        tra = tra + ",";
        tra = tra + QString::number(traj.at(i).at(1));
        tra = tra + ",";
        tra = tra + QString::number(traj.at(i).at(2));
        tra = tra + ");";
    }

    QString Id = QString::fromStdString(id);
    m_socketMpc->publishTopic("/robots/" + Id + "/trajectory", tra);
}

/**
 * @brief GeneralTopic::unsubscribeAll, don't need receive IDs, positions, targets anymore just before the optimization;
 * in case of re-initialize every time when the signal comes
 */
void GeneralTopic::unsubscribeAll()
{
    m_socketMpc->unsubscribe("/robots/all/#");
    m_socketMpc->unsubscribe("/robots/+/position");
    m_socketMpc->unsubscribe("/robots/+/targets");
    unsubscribedAll = true;
}

/**
 * @brief GeneralTopic::sendStop, when the robot reached its target, send this message
 * @param id, which robot has reached target
 */
void GeneralTopic::sendStop(const std::string &id)
{
    QString Id = QString::fromStdString(id);
    m_socketMpc->publishTopic("/robots/" + Id + "/stop", "");
}

/**
 * @brief GeneralTopic::getAllIDs, get the ID of all the robots
 * @return vector of IDs
 */
std::vector<std::string> GeneralTopic::getAllIDs()
{
    return m_robotsIDs;
}

/**
 * @brief GeneralTopic::getUnsubscribedBool, test whether unsubscribe the referred topic
 * @return boolean value
 */
bool GeneralTopic::getUnsubscribedBool()
{
    return unsubscribedAll;
}

void GeneralTopic::finish() {
    qDebug() << "SocketMPCThread finished" << endl;
}
