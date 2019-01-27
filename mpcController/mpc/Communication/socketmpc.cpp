#include "socketmpc.h"

/**
 * @brief Intialize the single instance of SocketMPC class
**/
SocketMPC* SocketMPC::m_instance = nullptr;

/**
 * @brief Returns the only instance of the class Endpoint (singleton).
 * @param brokerHostname: the Hostname of Brokers.
 * @param brokerPort: The available port of the broker for communication.
**/
SocketMPC* SocketMPC::getInstance(const QString& brokerHostname, const quint32 brokerPort)
{
    if(nullptr == m_instance)
    {
       m_instance = new SocketMPC(brokerHostname, brokerPort);
    }
    return m_instance;
}

/**
 * @brief Private constructor of the class SocketMPC (singleton).
 * @param brokerHostname: the Hostname of Brokers.
 * @param brokerPort: The available port of the broker for communication.
 */
SocketMPC::SocketMPC(const QString& brokerHostname, const quint32 brokerPort) :
    QMQTT::Client(brokerHostname, brokerPort)
{
    QObject::connect(this,SIGNAL(received(QMQTT::Message)),this,SLOT(receiv(QMQTT::Message)));
    connect();
}

void SocketMPC::receiv(QMQTT::Message msg){
    qDebug() << msg.topic();
}
/**
 * @brief Private destructor of the class Endpoint (singleton). Cleans up the created instance of the class Endpoint.
 */
SocketMPC::~SocketMPC()
{
    if(nullptr != m_instance)
    {
       delete m_instance;
       m_instance = nullptr;
    }
}

/**
 * @brief Connect to the broker.
 */
void SocketMPC::connectToBroker()
{
    connect();
}

/**
 * @brief SocketMPC::publishTopic, publish the topic
 * @param topic, topic name
 * @param payload, the content of the topic
 */
void SocketMPC::publishTopic(const QString& topic, const QString& payload)
{
    QMQTT::Message message;
    message.setId(qrand());
    message.setTopic(topic);
    message.setPayload(QByteArray(payload.toUtf8()));
    m_instance->publish(message);
}

/**
 * @brief Subscribes a topic with a definable Quality of Service.
 * @param topic: Topic classified hierarchically.
 * @param qos: Quality of Service.
 */
void SocketMPC::subscribeTopic(const QString& topic, quint8 qos)
{
    m_instance->subscribe(topic, qos);
}

/**
 * @brief Endpoint::setHostPort, set the host port
 * @param hostname
 * @param port
 */
void SocketMPC::setHostPort(const QString &hostname, const quint32 port)
{
    setHost(hostname);
    setPort(port);
}
