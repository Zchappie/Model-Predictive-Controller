#ifndef SOCKETCLIENT_H
#define SOCKETCLIENT_H

#include <QtNetwork/QHostAddress>
#include <QTcpSocket>
#include <QtCore/QDataStream>
#include <QtCore/QPointer>
#include <QtCore/QObject>

#include <nlopt.hpp>

#include <constraint.h>
#include <CostFunction.h>

#include <vector>

/**
 * @brief The SocketClient class, send calculation request to the server
 */
class SocketClient : public QObject
{
public:
    SocketClient();

    std::vector<std::vector<double> > sendRequest(const nlopt::opt& opt, const std::vector<std::vector<double> >& control,
                                                  const std::vector<Constraint>& constraints, const CostFunction &costFunction,
                                                  const std::vector<double> x0, const std::shared_ptr<Model> &model);
    bool connected() const;
private:
    QPointer<QTcpSocket> m_socket;
    QDataStream m_out;
    bool m_isConnected;
};

#endif // SOCKETCLIENT_H
