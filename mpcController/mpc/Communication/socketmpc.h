#ifndef SOCKETMPC_H
#define SOCKETMPC_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include "../qmqtt/qmqtt_client.h"

/// broker parameters
/// change here the QMQTT server IP address!!!
static constexpr const char* BROKER_HOST = "192.168.2.101";
static const quint32 BROKER_PORT = 1883;

/**
 * @brief The SocketMPC class, a singleton client communicates with broker to publish and subscribe topics
 * customerized from the QMQTT library
 */
class SocketMPC : public QMQTT::Client
{
    Q_OBJECT

public:
    /// get a single instance of endpoint
    static SocketMPC* getInstance(const QString& brokerHostname=BROKER_HOST, const quint32 brokerPort=BROKER_PORT);

    /**
     * guaranteed levels of Quality Of Service:
          low  = message will be delivered at most once (best effort delivery)
          mid  = message will be delivered at least once
          high = message will be delivered exactly once
    */
    enum QualityOfServiceLevel {Low = 0, Mid = 1, High = 2};

    /// !!! breaking encapsulation !!!
    ~SocketMPC();

private:
    /// for singleton design pattern - set constructor, copy Constructor and '= op' as private

    SocketMPC(const QString& brokerHostname=BROKER_HOST, const quint32 brokerPort=BROKER_PORT); /// constructor
    SocketMPC(const SocketMPC& other); /// copy constructor
    SocketMPC& operator=(SocketMPC& other); /// = operator


public slots:
    void receiv(QMQTT::Message msg);
    /// connect to broker
    void connectToBroker();

    /// publish a topic
    void publishTopic(const QString& topic, const QString& payload);

    /// subscribe a topic
    void subscribeTopic(const QString& topic, quint8 qos);

    void setHostPort(const QString& hostname, const quint32 port);

private:
    /// the only one instance of endpoint
    static SocketMPC* m_instance;
    QTimer *timer;
};

#endif // SOCKETMPC_H
