#include <QCoreApplication>
#include <iostream>

#include <QDebug>
#include <chrono>
#include <thread>
#include <functional>
#include <vector>
#include "Communication/generaltopic.h"

/**
 * @brief main of MPC control process
 * @return
 */
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    a.setObjectName("MainThread");

    GeneralTopic gt;
    return a.exec();
}


