#ifndef NLOPTNETWORK_H
#define NLOPTNETWORK_H

#include "nloptthread.h"

#include <QtNetwork/QTcpServer>
#include <QtCore/QList>
#include <QtCore/QObject>

//Identification of the client: via socket (?)
//Input: what to serialize: Nlopt object, control vector as input (own operator ?), size of control vector (?)
//Output: control vector as solution

/**
 * @brief The NLOptNetwork class serialize an Nlopt object and the imposed vector
 */
class NLOptNetwork : public QTcpServer
{
    Q_OBJECT
public:
    NLOptNetwork();
signals:
    void receiveRequest();
public slots:
protected:
    void incomingConnection(qintptr socketDescriptor) override;
private slots:
    void deleteFromList(const QPointer<NLOptThread> &nloptThread);
private:
    QList<QPointer<NLOptThread> > m_socketThreads;
};



#endif // NLOPTNETWORK_H
