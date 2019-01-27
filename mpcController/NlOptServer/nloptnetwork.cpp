#include "nloptnetwork.h"

#include <QtCore/QMetaType>

NLOptNetwork::NLOptNetwork()
{
    qRegisterMetaType<QPointer<NLOptThread>>("QPointer<NLOptThread>");
    listen(QHostAddress::Any, 64500);
}

void NLOptNetwork::incomingConnection(qintptr socketDescriptor) {
    QPointer<NLOptThread> nloptThread = new NLOptThread(socketDescriptor, this);
    if (nloptThread) {
        connect(nloptThread, SIGNAL(finished()), nloptThread, SLOT(deleteLater()));
        connect(nloptThread, SIGNAL(finishedWithAddress(const QPointer<NLOptThread>&)), this, SLOT(deleteFromList(const QPointer<NLOptThread>&)));
        m_socketThreads.append(nloptThread);
        nloptThread->start();
    }
}
void NLOptNetwork::deleteFromList(const QPointer<NLOptThread>& nloptThread) {
    for (int i = 0; i < m_socketThreads.size(); i++) {
        if (nloptThread == m_socketThreads.at(i)) {
            m_socketThreads.at(i)->quit();
            m_socketThreads.at(i)->wait();
            m_socketThreads.removeAt(i);
            break;
        }
    }
}

