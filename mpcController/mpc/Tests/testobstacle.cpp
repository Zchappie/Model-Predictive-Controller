#include "testobstacle.h"

TestObstacle::TestObstacle(const double& t):
    m_t(t)
{
    m_N = horizon;
    m_T = sampleInterval;
}

void TestObstacle::setActual(const double& t0)
{
    m_t0 = t0;
}

std::vector<std::vector<double> > TestObstacle::calcTraj(const std::vector<double>& control)
{
    ///first, calculate the trajectory
    std::vector<std::vector<double> > trajectory;
    std::vector<std::vector<double> > uReshaped = VectorHelper::reshapeXd(control, controlVecSize);
    trajectory.push_back({0.0,0.0,0.0});
    Model model;
    for (size_t i = 0; i < m_N; i++)
    {
        trajectory.push_back(model.calcNext(trajectory.back(), uReshaped.at(i)));
    }
    for (size_t i = 0; i < m_N+1; i++)
    {
        trajectory[i].erase(trajectory[i].begin()+2); //erase the angular position
    }
}

double TestObstacle::wrapTestObject(const std::vector<double>& u, std::vector<double>& grad, void* data)
{
    qDebug("Wrapping the TestObstacle constraint object...");
    return (*reinterpret_cast<TestObstacle*>(data)) (u, grad);
}

/**
 * @brief TestObstacle::operator () add an obstacle as single constraint to test the mpccontroller, haven't succeed yet
 * @param control
 * @param grad
 * @param f_data
 * @return
 */
double TestObstacle::operator()(const std::vector<double> &control, std::vector<double> &grad, void* f_data)
{
    qDebug("Test Obstacle is adding...");
    //calculate the trajectory
    std::vector<std::vector<double> > trajectory;
    trajectory = calcTraj(control);

    // get the correct position of the constraint
    int pos = (m_t - m_t0) / (double)(m_T);
    Q_ASSERT_X(pos >= 0, typeid(this).name() ,"pos is negative");

    //the car should outside the minDistance related to the obstacle
    std::vector<double> obstacle = {0.6, 0};
    double distance = sqrt(pow((trajectory.at(pos).at(0)-obstacle[0]),2) + pow((trajectory.at(pos).at(1)-obstacle[1]),2)) - minDistance;
    return (-1.0*distance);
}
