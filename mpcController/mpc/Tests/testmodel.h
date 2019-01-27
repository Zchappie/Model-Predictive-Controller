#ifndef TESTMODEL_H
#define TESTMODEL_H


#include <QtTest/QtTest>
#include <QtCore/QObject>
#include "../model.h"

/**
 * @brief The TestModel class to test Model class, mainly to test the calcNext function
 */
class TestModel : public QObject
{
public:
    Q_OBJECT

private slots:
    void TestCalcNext();

private:

};


#endif // TESTMODEL_H
