#include "testcostfunction.h"

#include <memory>

void TestCostFunction::TestCostOperator()
{
    //time
    double t = 0.0;
    //positions
    std::vector<double> target = {10, 0, 0};

    //model
    std::shared_ptr<Model> model = std::make_shared<Model>();
    std::vector<double> startPos = {0.8, 0, 0};
    model->setStart(startPos);

    //initialize the costFunction instance
    CostFunction cost(startPos, target, t);
    std::vector<double> control = {0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0,0.5, 0};
    std::vector<double> grad;

    //the summation of the stage costs
    double costs = cost.operator ()(control, grad);
    QCOMPARE(costs, 472.78);
}
