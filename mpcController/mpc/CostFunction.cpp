#include "CostFunction.h"
#include "constparameter.h"

CostFunction::CostFunction() :
    m_x0(0.0),
    m_target(std::vector<double>()),
    m_t0(0.0)
{

}

/**
 * @brief CostFunction::CostFunction, constructor of Costfunction class
 * @param x0, current state
 * @param target
 * @param t0
 */
CostFunction::CostFunction(const std::vector<double> &x0, const std::vector<double> &target, const double& t0) :
    m_x0(x0),
    m_target(target),
    m_t0(t0)
{
    //qDebug() << "[CostFunction] Initializing the CostFunction class...";
    m_T = sampleInterval;
    m_N = horizon;
    m_lambda = lambda;
}

/**
 * @brief CostFunction::getStageCosts, parameter for y is chosen as 3.0, as the detour will not take too long
 * @param x, state of a certain future point
 * @param u, control of a certain future point
 * @return costs, the stage costs
 */
double CostFunction::getStageCosts(const std::vector<double>& x, const std::vector<double>& u) const
{
    double costs = 0.0;
    ///first part, certain point to target
    std::vector<double> state_cost;
    state_cost.push_back(pow(x[0]-m_target[0], 2.0));
    state_cost.push_back(5.0*(x[1]-m_target[1])); ///the parameter changed from 5.0 to 3.0
    state_cost.push_back(pow(x[2]-m_target[2], 2.0));

    ///second part
    std::vector<double> control_cost;
    control_cost.push_back(pow(u[0],2));
    control_cost.push_back(pow(u[1],2));

    ///all together
    costs = VectorHelper::norm2(state_cost) + m_lambda*VectorHelper::norm2(control_cost);
    return costs;
}

/**
 * @brief CostFunction::operator () evaluate the costs for a given trajectory, which is starting at a certain point (m_x0)
 * @param u
 * @param grad
 * @param f_data
 * @return
 */
double CostFunction::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data)
{
    //qDebug("[CostFunction] Starting the operator()...");
    double costs = 0.0;
    ///first, calculate the trajectory
    std::vector<std::vector<double> > trajectory;
    trajectory.push_back(m_x0);
    std::vector<std::vector<double> > uReshaped = VectorHelper::reshapeXd(u, controlVecSize);
    Model model;

    for (size_t i = 0; i < m_N; i++)
    {
        trajectory.push_back(model.calcNext(trajectory.back(), uReshaped.at(i)));
        //qDebug()<<"[CostFunction]" << i+1 << "trajectory is" << trajectory[i+1][0] << trajectory[i+1][1] << trajectory[i+1][2];
    }
    for (size_t i = 0; i < m_N+1; i++)
    {
        trajectory[i].erase(trajectory[i].begin()+2); ///erase the angular position
    }
    //std::vector<std::vector> > x = systemFunc->getTrajectory(m_x0, u, t0);

    ///calculate the costs
    std::vector<double> traj_help(doF);
    std::vector<double> u_help(doF);

    for(size_t i = 1; i < m_N+1; i++)
    {
        traj_help[0] = trajectory[i][0];
        traj_help[1] = trajectory[i][1];

        u_help[0] = uReshaped[i-1][0];
        u_help[1] = uReshaped[i-1][1];
        double costs_mid = getStageCosts(traj_help, u_help);
        //qDebug() << "[CostFunction] The " <<i<< "th stage cost..." << "is "<< costs_mid;
        costs += costs_mid;
    }
    //qDebug() << "costs of pos(" << m_x0.at(0) << "," << m_x0.at(1) << ") to ("<< m_target.at(0) << "," << m_target.at(1) << ")" << "is " << costs << endl;
    return costs;
}

/**
 * @brief CostFunction::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param u control value vector
 * @param grad gradient (if available)
 * @param data (additional data)
 * @return
 */
double CostFunction::wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data)
{
    return (*reinterpret_cast<CostFunction*>(data)) (u, grad);
}

/**
 * @brief operator << writes the cost function into a datastream with the following format
 * (sizeof(x_0), x0, t0, T, N, lambda)
 * @param out
 * @param costFunction
 * @return
 */
QDataStream& operator<<(QDataStream& out, const CostFunction& costFunction) {
    out << (quint16)costFunction.m_x0.size();
    for (size_t i = 0; i < costFunction.m_x0.size(); i++) {
        out << costFunction.m_x0.at(i);
    }
    for (size_t i = 0; i < costFunction.m_target.size(); i++) {
        out << costFunction.m_target.at(i);
    }
    out << costFunction.m_t0;
    out << costFunction.m_T;
    out << (quint16)costFunction.m_N;
    out << costFunction.m_lambda;
    return out;
}

/**
 * @brief operator >> creates the cost function from a stream, parameter and order are
 * (sizeof(x_0), x0, t0, T, N, lambda)
 * @param in
 * @param costFunction
 * @return
 */
QDataStream& operator>>(QDataStream& in, CostFunction& costFunction) {
    size_t N = 0, vectorSize = 0;
    quint16 qN = 0, qVectorSize = 0;
    in >> qVectorSize;
    vectorSize = (size_t)qVectorSize;
    for (size_t i = 0; i < vectorSize; i++) {
        double curX0;
        in >> curX0;
        costFunction.m_x0.push_back(curX0);
    }
    for (size_t i = 0; i < vectorSize; i++) {
        double curTarget;
        in >> curTarget;
        costFunction.m_target.push_back(curTarget);
    }
    in >> costFunction.m_t0;
    in >> costFunction.m_T;
    in >> qN;
    costFunction.m_N = qN;
    in >> costFunction.m_lambda;
    return in;
}
