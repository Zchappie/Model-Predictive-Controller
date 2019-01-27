#include "Tests/testconstraint.h"

/**
 * @brief TestConstraint::TestOperator test the operator() function
 */
void TestConstraint::TestOperator()
{
    //time
    double t = 0.0;
    //positions
    std::vector<double> otherPos = {1.5, 0};

    //model
    std::shared_ptr<Model> model = std::make_shared<Model>();
    std::vector<double> startPos = {0.8, 0, 0};
    model->setStart(startPos);
    //constraints
    //initialize the constraint instance
    Constraint constraint(t, otherPos);
    constraint.setActualSystem(model, t);
    std::vector<double> control = {0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0};
    std::vector<double> grad;

    //the inequality is sent to optimize, here cant say > or < than 0
    bool smallZero = constraint.operator ()(control, grad) <= 0;
    QCOMPARE(smallZero, true);
}
