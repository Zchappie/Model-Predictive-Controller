#ifndef TESTCOSTFUNCTION_H
#define TESTCOSTFUNCTION_H

#include <QtTest/QtTest>
#include <QtCore/QObject>

#include "../CostFunction.h"

/**
 * @brief The TestCostFunction class, test the main method "operator()" in CostFunction class
 */
class TestCostFunction : public QObject
{

public:
    Q_OBJECT

private slots:
    void TestCostOperator();

private:
};

#endif // TESTCOSTFUNCTION_H
