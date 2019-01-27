#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <cmath>
#include <QDebug>


/**
 * @brief The Model class, model the kinematics of the robot
 */
class Model
{
public:
    Model();
    Model(const std::vector<double>& x0, const double& T);
    std::vector<double> calcNext(const std::vector<double>& x0, const std::vector<double>& u);
    void setStart(const std::vector<double>& x0);
    std::vector<double> start() const;
    double getSampleInterval() const;

    friend QDataStream& operator<<(QDataStream& out, const Model& model);
    friend QDataStream& operator>>(QDataStream& in, Model& model);

private:
    ///current position (x,y,orientation)
    std::vector<double> m_x0;
    ///sample interval
    double m_T;
};

#endif // MODEL_H
