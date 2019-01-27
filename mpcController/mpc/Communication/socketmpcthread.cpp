#include "socketmpcthread.h"

#include <QtCore/QMetaType>

SocketMpcThread::SocketMpcThread(const QString& brokerHostname, const quint32 brokerPort):QThread()
{
    bhn = brokerHostname;
    bp = brokerPort;
}

/**
 * @brief Wird beim Start des Threads ausgeführt. Verbindet alle Signale und Slots vom SocketMPC mit der
 *        SocketMpcThread Klasse.
 */
void SocketMpcThread::run()
{
    /// Enthält den SocketMPC des Threads
    QPointer<SocketMPC> mpc = SocketMPC::getInstance(bhn,bp);
    qRegisterMetaType<QMQTT::Message>("QMQTT::Message");
    QObject::connect(this,SIGNAL(tPubMessage(QString,QString)),mpc,SLOT(publishTopic(QString,QString)));
    QObject::connect(this,SIGNAL(tsetHostPort(QString,quint32)),mpc,SLOT(setHostPort(QString,quint32)));
    QObject::connect(this,SIGNAL(tsubTopic(QString,quint8)),mpc,SLOT(subscribeTopic(QString,quint8)));
    QObject::connect(mpc,SIGNAL(received(QMQTT::Message)),this,SLOT(receiv(QMQTT::Message)));
    QObject::connect(mpc,SIGNAL(connected()),this,SLOT(tConnected()));
    QObject::connect(this,SIGNAL(tConnectToBroker()),mpc,SLOT(connectToBroker()));
    QObject::connect(this,SIGNAL(tUnsubscribe(QString)),mpc,SLOT(unsubscribe(QString)));
    this->setObjectName("SocketMPCThread");
    qDebug()<<"[SocketMPCThread] NAME:"<<QObject::objectName();
    this->exec();
    qDebug() << "SocketMPCThread exited" << endl;
}

void SocketMpcThread::unsubscribe(const QString& unsub){
    emit tUnsubscribe(unsub);
}

/**
 * @brief Leitet das Signal weiter an SocketMPCt.
 * @param topic Emittiert das Topic weiter.
 * @param payload Emittiert das Payload weiter.
 */
void SocketMpcThread::publishTopic(const QString &topic, const QString &payload){
    emit(tPubMessage(topic,payload));
}

/**
 * @brief Leitet das Signal weiter an SocketMPC.
 * @param topic Emittiert das Topic weiter.
 * @param qos Emittiert den Inhalt des Topics weiter.
 */
void SocketMpcThread::subscribeTopic(const QString &topic, quint8 qos){
    emit(tsubTopic(topic,qos));
}

/**
 * @brief Leitet das Signal weiter an SocketMPC.
 * @param host Emittiert Host weiter.
 * @param port Emittiert den Port weiter.
 */
void SocketMpcThread::setHostPort(const QString &host, const quint32 port){
    emit(tsetHostPort(host,port));
}

/**
 * @brief Leitet das Signal aus SocketMPC.
 * @param msg Emittiert die Message, bestehend aus Topic und Payload weiter.
 */
void SocketMpcThread::receiv(QMQTT::Message msg){
    emit(received(msg));
}

/**
 * @brief Leitet das Signal aus SocketMPC.
 */
void SocketMpcThread::tConnected(){
    emit(connected());
}

/**
 * @brief Leitet das Signal weiter an SocketMPC.
 */
void SocketMpcThread::connect(){
    emit(tConnectToBroker());
}

/**
 * @brief Leitet das Signal weiter an SocketMPCt.
 * @param host Setzt den Host des SocketMPCs auf die gewünschte Eingabe.
 */
/*void SocketMpcThread::setHost(const QString &host)
{
    mpc->setHost(host);
}*/

/**
 * @brief Leitet das Signal weiter an SocketMPC.
 * @param port Setzt den Host des SocketMPCs auf die gewünschte Eingabe.
 */
/*void SocketMpcThread::setPort(quint32 port)
{
    mpc->setPort(port);
}*/

