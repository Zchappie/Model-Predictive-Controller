#ifndef TESTCONSTRAINT_H
#define TESTCONSTRAINT_H

#include <QtTest/QtTest>
#include <QtCore/QObject>

#include "../constraint.h"

/**
 * @brief The TestConstraint class to test the constraint class, mainly tests the operator() function
 */
class TestConstraint : public QObject
{
public:
    Q_OBJECT

private slots:
    void TestOperator();

private:

};



#endif // TESTCONSTRAINT_H
