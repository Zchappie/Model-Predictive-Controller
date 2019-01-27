#include "socketclient.h"

SocketClient::SocketClient() :
    m_isConnected(false)
{
    m_socket = new QTcpSocket(this);
    //INSERT IP address of workstation where NLOptServer is running
    m_socket->connectToHost("192.168.2.101", 64500);
    if (!m_socket->waitForConnected(100))
    {
        qDebug() << typeid(this).name() << "Connection error: " << m_socket->errorString() << endl;
        qDebug() << "switch to local optimisation" << endl;
        //return;
    }
    else {
        m_isConnected = true;
    }
    if (m_socket) {
        m_out.setDevice(m_socket);
    }
}
/**
 * @brief SocketClient::sendRequest Format is (VectorLength N, vectordimension M, controlvector (N - 1), statevectordimension P, x0 (P), constraints.size()
 * Constraints, CostFunction, lower bound, upper bound, Model (current robot, which optimise), t (current time)
 *
 * @param opt Optimiser Problem (OCP)
 * @param control control values u(0)...u(N-1)
 * @param constraints all constraints concerning the problem
 * @param costFunction F(x) of the robot
 * @param x0 current state of the robot
 * @param model Model of the robot
 * @return result of control vector
 */
std::vector<std::vector <double> > SocketClient::sendRequest(const nlopt::opt& opt, const std::vector<std::vector<double> > &control, const std::vector<Constraint>& constraints, const CostFunction& costFunction,
                               const std::vector<double> x0, const std::shared_ptr<Model>& model)
{
    std::vector<std::vector<double> > controlResult;
    if (m_socket && m_socket->isWritable() && m_socket->state() == QAbstractSocket::ConnectedState)
    {
        //write control vector
        size_t vectorLength = control.size(), vectorControlDim = control.front().size();
        quint16 qVectorLength = (quint16)vectorLength, qVectorDim = (quint16)vectorControlDim;
        m_out << qVectorLength;
        m_out << qVectorDim;
        for (size_t i = 0; i < control.size(); i++)
        {
            for (size_t j = 0; j < control.at(i).size(); j++)
            {
                m_out << control.at(i).at(j);
            }
        }
        //write start vector
        m_out << qVectorLength;
        m_out << qVectorDim;
        m_out << (quint16)x0.size();
        for (size_t i = 0; i < x0.size(); i++)
        {
            m_out << x0.at(i);
        }
        //write constraints
        m_out << (quint16)constraints.size();
        for (const Constraint& constraint : constraints)
        {
            m_out << constraint;
        }
        //write cost function
        m_out << costFunction;
        //TOBIAS: please check, if a vector should be transmitted for lower/upper bounds or only one value
        std::vector<double> lb = opt.get_lower_bounds();
        std::vector<double> ub = opt.get_upper_bounds();
        m_out << lb.front();
        m_out << ub.front();
        m_out << model.get();
        m_socket->flush();
        //after written, wait for read
        m_socket->waitForBytesWritten(10000);
        m_socket->waitForReadyRead(-1);
        for (size_t i = 0; i < control.size(); i++)
        {
            for (size_t j = 0; j < control.front().size(); j++)
            {
                if (controlResult.size() <= i)
                {
                    controlResult.push_back(std::vector<double>());
                }
                double resControl = 0.0;
                m_out >> resControl;
                controlResult.at(i).push_back(resControl);
            }
        }
    }
    return controlResult;
}

bool SocketClient::connected() const {
    return m_isConnected;
}
