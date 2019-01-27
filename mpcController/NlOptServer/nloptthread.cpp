#include "nloptthread.h"

#include "../../mpcController/mpc/constraint.h"
#include "../../mpcController/mpc/CostFunction.h"

#include <iostream>


NLOptThread::NLOptThread(int socketDescriptor, QObject *parent) :
    QThread(parent),
    m_socketDescriptor(socketDescriptor)
{
    //
}

void NLOptThread::run() {
    QTcpSocket socket;
    //socket->moveToThread(this);
    if (!socket.setSocketDescriptor(m_socketDescriptor)) {
        emit error(socket.error());
        return;
    }
    readFromSocket(socket);
    emit finishedWithAddress(this);
    //connect(socket, SIGNAL(readyRead()), this, SLOT(readFromSocket()));
}

void NLOptThread::readFromSocket(QTcpSocket& socket) {
    socket.waitForReadyRead(-1);
    QDataStream in;
    in.setDevice(&socket);
    long long bytesAvail = socket.bytesAvailable();
    //in.startTransaction();
    if (bytesAvail > 0) {
        //read in control vector
        size_t vectorLength = 0, vectorControlDim = 0;
        quint16 qVectorLength = 0, qVectorDim = 0;
        in >> qVectorLength;
        in >> qVectorDim;
        vectorLength = (size_t)qVectorLength;
        vectorControlDim = (size_t)qVectorDim;
        std::vector<std::vector<double> > controlVector;
        controlVector.reserve(vectorLength);
        for (size_t i = 0; i < vectorLength; i++) {
            for (size_t j = 0; j < vectorControlDim; j++) {
                if (controlVector.size() <= i) {
                    controlVector.push_back(std::vector<double>());
                }
                double curVal = 0.0;
                in >> curVal;
                controlVector.at(i).push_back(curVal);
            }
        }
        std::cout << typeid(this).name() << "readFromSocket" << std::endl;
        std::cout << "Vector length: " << vectorLength << std::endl;
        std::cout << "Vector control dim: " << vectorControlDim << std::endl;
        //read in boundaries, constraints, cost function, for NLOpt
        //in >> optObject;
        //read in parameters control vector
        //size_t vectorLength = 0, vectorControlDim = 0;
        //quint16 qVectorLength = 0, qVectorDim = 0;
        in >> qVectorLength;
        in >> qVectorDim;
        vectorLength = (size_t)qVectorLength;
        vectorControlDim = (size_t)qVectorDim;

        size_t stateVectorDim = 0;
        quint16 qStateVectorDim = 0;
        in >> qStateVectorDim;
        stateVectorDim = (size_t)qStateVectorDim;
        std::vector<double> x0;
        for (size_t i = 0; i < stateVectorDim; i++) {
            double curX;
            in >> curX;
            x0.push_back(curX);
        }
        quint16 numberConstraints = 0;
        in >> numberConstraints;
        std::vector<Constraint> constraints;
        for (size_t i = 0; i < numberConstraints; i++) {
            Constraint constraint;
            in >> constraint;
            constraints.push_back(constraint);
        }
        CostFunction costFunction;
        in >> costFunction;
        double lb = 0.0, ub = 0.0;
        in >> lb;
        in >> ub;
        Model modelIn;
        in >> modelIn;
        double t0;
        in >> t0;
        std::shared_ptr<Model> model = std::make_shared<Model>(modelIn.start(), modelIn.getSampleInterval());
        //TODO: create nlopt object
        std::cout << "nlopt::operator>>" << "start optimisation" << std::endl;
        nlopt::opt nlOpt = nlopt::opt(nlopt::LN_COBYLA, vectorLength * vectorControlDim);
        for (Constraint& constraint : constraints) {
            //TODO: serialize class model here
            constraint.setActualSystem(model,t0);
            nlOpt.add_inequality_constraint(Constraint::wrapConstraintFunctionObject, &constraint);
        }
        nlOpt.set_min_objective(CostFunction::wrapCostFunctionObject, &costFunction);
        nlOpt.set_lower_bounds(lb);
        nlOpt.set_upper_bounds(ub);
        //end transaction, if not valid, reject and begin new
        //if (!in.commitTransaction()) {
        //    return;
        //}
        double functionVal;
        std::vector<double> oneDimVector = VectorHelper::reshapeXdTo1d(controlVector);
        nlopt::algorithm alg = nlOpt.get_algorithm();
        try {nlopt::result res = nlOpt.optimize(oneDimVector, functionVal);}
        catch (nlopt::roundoff_limited) {
            qDebug() << typeid (this).name() << "round_off limit" << endl;
        }
        std::cout << typeid (this).name() << "end of optimisation" << std::endl;
        //write to socket the result
        controlVector.clear();
        controlVector = VectorHelper::reshapeXd(oneDimVector);
        QByteArray bufferOut;
        QDataStream out(&bufferOut, QIODevice::WriteOnly);
        //TODO: evaluate, if this is the correct method to see, if socket is free to write
        std::cout << typeid (this).name() << "start to write back" << std::endl;
        if (socket.bytesAvailable() == 0) {
            for (size_t i = 0; i < controlVector.size(); i++) {
                for (size_t j = 0; j < controlVector.at(i).size(); j++) {
                    out << controlVector.at(i).at(j);
                }
            }
            socket.write(bufferOut);
            socket.waitForBytesWritten();
        }
        std::cout <<typeid (this).name() << "end of write back" << std::endl;
    }
}

/**
 * @brief operator >> initialise and fill in an
 * Format is (VectorLength N, vectordimension M, controlvector (N - 1), statevectordimension P, x0 (P),
 * Constraints.size(), Constraints, CostFunction, lower bound, upper bound, Model (current robot, which optimise), t (current time)
 * @param in
 * @param nlOpt
 * @return
 */
QDataStream& operator>>(QDataStream& in, nlopt::opt& nlOpt) {

    //nlOpt = *nlOptObj;
}
