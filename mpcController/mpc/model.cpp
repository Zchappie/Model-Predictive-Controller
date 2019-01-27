#include "model.h"
#include "constparameter.h"

#include <QtCore/QDataStream>

/**
 * @brief Model::Model
 * @param x0
 */
Model::Model()
{
    m_T = sampleInterval;
}

Model::Model(const std::vector<double>& x0, const double& T) :
    m_x0(x0),
    m_T(T)
{

}

/**
 * @brief Model::setStart set the start state
 * @param x0
 */
void Model::setStart(const std::vector<double>& x0)
{
    m_x0 = x0;
}

/**
 * @brief Model::start
 * @return
 */
std::vector<double> Model::start() const
{
    return m_x0;
}

double Model::getSampleInterval() const {
    return m_T;
}

/**
 * @brief calcNext, calculate the next state of the robot car, using the control commands
 * @param x0, current state
 * @param u, control command, up1 linear velocity, up2 angular velocity
 * @param T, sampling time interval
 * @return the next state of the robot car, vector (x, y, theta(commented out))
 */
std::vector<double> Model::calcNext(const std::vector<double>& x0, const std::vector<double>& u)
{
    std::vector<double> xNext;
    xNext.push_back(x0.at(0) + m_T * std::cos(x0.at(2)) * u.at(0));
    xNext.push_back(x0.at(1) + m_T * std::sin(x0.at(2)) * u.at(0));
    xNext.push_back(x0.at(2) + m_T * u.at(1));
    return xNext;
}

/**
 * @brief operator << serialise the class Model to a QDataStream, format is (vectorLength, vector (x0), T)
 * @param out QDataStream to write out
 * @param model instance to serialise
 * @return  QDataStream to write
 */
QDataStream& operator<<(QDataStream& out, const Model& model) {
    quint16 vectorLength = (quint16)model.m_x0.size();
    out << vectorLength;
    for (size_t i = 0; i < model.m_x0.size(); i++) {
        out << model.m_x0.at(i);
    }
    out << model.m_T;
    return out;
}

/**
 * @brief operator >> reads in a model with (vectorLength, vector, T)
 * @param in
 * @param model
 * @return
 */
QDataStream& operator>>(QDataStream& in, Model& model) {
    model = Model();
    quint16 vectorLength;
    in >> vectorLength;
    std::vector<double> x0;
    for (size_t i = 0; i < (size_t)vectorLength; i++) {
        double curX;
        in >> curX;
        x0.push_back(curX);
    }
    model.m_x0 = x0;
    in >> model.m_T;
}
