#ifndef NLOPTTHREAD_H
#define NLOPTTHREAD_H

#include "nlopt.hpp"

#include <QtNetwork/QTcpSocket>
#include <QtCore/QThread>
#include <QtCore/QDataStream>
#include <QtCore/QPointer>

class NLOptThread : public QThread
{
    Q_OBJECT
public:
    NLOptThread(int socketDescriptor, QObject* parent);
    void run() override;
public slots:
    void readFromSocket(QTcpSocket &socket);
signals:
    void error(QTcpSocket::SocketError socketError);
    void finishedWithAddress(const QPointer<NLOptThread>&);
private slots:

private:
    //QPointer<QTcpSocket> socket;
    int m_socketDescriptor;
};

QDataStream& operator>>(QDataStream& in, nlopt::opt& nlOpt);

#endif // NLOPTTHREAD_H
