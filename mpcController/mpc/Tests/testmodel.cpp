#include "Tests/testmodel.h"

void TestModel::TestCalcNext()
{
    //1.initialize
    std::vector<double> state = {1,1,0};
    std::vector<std::vector<double> > control = {{0.5, 0}, {0.5, 0},{0.5, 0}};
    std::vector<double> nextState(2);

    //2.calculate
    Model model;
    nextState = model.calcNext(state, control[0]);

    //3.compare
    std::vector<double> answer = {1.25, 1};
    QCOMPARE(nextState, answer);
}
