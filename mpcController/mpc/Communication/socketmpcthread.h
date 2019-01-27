#ifndef SOCKETMPCTHREAD_H
#define SOCKETMPCTHREAD_H

#include <QThread>
#include "Communication/socketmpc.h"

class SocketMpcThread : public QThread
{
    Q_OBJECT

public:
    explicit SocketMpcThread(const QString& brokerHostname=BROKER_HOST, const quint32 brokerPort=BROKER_PORT);
    void run();
    void connect();
    //void setHost(const QString &host);
    //void setPort(quint32 port);

public slots:
    void receiv(QMQTT::Message msg);
    void publishTopic(const QString& topic, const QString& payload);
    void subscribeTopic(const QString& topic, quint8 qos);
    void setHostPort(const QString& host,const quint32 port);
    void tConnected();
    void unsubscribe(const QString& unsub);

signals:
    void tUnsubscribe(const QString& unsub);
    /**
     * @brief Sendet das Signal weiter zum Receiveslot des SocketMPCs
     * @param msg Nachricht die versendet werden soll
     */
    void received(QMQTT::Message msg);
    /**
     * @brief Sendet Topic und Payload weiter zum Publishslot des SocketMPCs
     * @param topic Das Topic, welches weitergeleitet wird
     * @param payload Die Payload, welche weitergeleitet wird
     */
    void tPubMessage(const QString& topic, const QString& payload);
    /**
     * @brief Sendet Topic und qos weiter zum Subscribeslot des SocketMPCs
     * @param topic Das Topic welches versendet wird
     * @param qos Die qos welches versendet wird
     */
    void tsubTopic(const QString& topic, quint8 qos);
    /**
     * @brief Sendet Host und Port weiter zum setHostPortSlot des SocketMPCs
     * @param host Der Host welcher versendet wird
     * @param port Der Port welcher versendet wird
     */
    void tsetHostPort(const QString &host, const quint32 port);
    /**
     * @brief Verbindet sich mit dem tConnectedSlot des SocketMPCs
     */
    void connected();
    /**
     * @brief Verbindet sich mit dem connectToBrokerSlot des SocketMPCs
     */
    void tConnectToBroker();
private:
    ///brocker host name
    QString bhn;

    ///brocker port
    quint32 bp;
};


#endif // SocketMPCTHREAD_H
