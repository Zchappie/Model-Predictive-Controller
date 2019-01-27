
#include "nloptnetwork.h"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    NLOptNetwork networkServer;
    return a.exec();
}

