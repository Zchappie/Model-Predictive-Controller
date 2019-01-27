QT += core
QT -= gui
QT += network
QT += testlib

QMAKE_CXXFLAGS += -std=c++11 -Wall -Wextra -pedantic -g

TARGET = mpc
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

#comment this out to ignore the qdebug output
#DEFINES += QT_NO_DEBUG_OUTPUT

SOURCES += main.cpp \
    CostFunction.cpp \
    mpccontroller.cpp \
    constraint.cpp \
    vectorhelper.cpp \
    cell.cpp \
    model.cpp \
    controlconstraint.cpp \
    robot.cpp \
    Tests/testmodel.cpp \
    Tests/testconstraint.cpp \
    Tests/testcostfunction.cpp \
    initialization.cpp \
    socketclient.cpp \
    Communication/socketmpc.cpp \
    Communication/generaltopic.cpp \
    constparameter.cpp \
    Communication/socketmpcthread.cpp \
    worker.cpp \
    controller.cpp



HEADERS += \
    CostFunction.h \
    mpccontroller.h \
    constraint.h \
    vectorhelper.h \
    cell.h \
    model.h \
    controlconstraint.h \
    robot.h \
    Tests/testconstraint.h \
    Tests/testmodel.h \
    Tests/testcostfunction.h \
    initialization.h \
    socketclient.h \
    Communication/socketmpc.h \
    Communication/generaltopic.h \
    constparameter.h \
    Communication/socketmpcthread.h \
    worker.h \
    controller.h

INCLUDEPATH += "$$PWD/../qmqtt"
INCLUDEPATH += "$$PWD/../nlopt"

win32:CONFIG(release, debug|release): LIBS += -L"$$OUT_PWD/../qmqtt/debug" -lqmqtt
else:win32:CONFIG(debug, debug|release): LIBS += -L"$$OUT_PWD/../qmqtt/debug" -lqmqtt
else:unix: LIBS += -L"$$OUT_PWD/../qmqtt/" -lqmqtt



win32:CONFIG(release, debug|release): LIBS += -L"$$PWD/../nlopt/libs/win" -lnlopt-0
else:win32:CONFIG(debug, debug|release): LIBS += -L"$$PWD/../nlopt/libs/win" -lnlopt-0
else:unix: LIBS += -L$$PWD/../nlopt/libs/ -lnlopt

#common outdir for all projects
OUTDIR = $$OUT_PWD/../out

#make dir for outdir
#QMAKE_PRE_LINK = $(MKDIR)  $$OUT_PWD/../../out/
#copy binaries to common outdir
#QMAKE_POST_LINK += $(COPY) $$OUT_PWD/$$TARGET $$OUT_PWD/../../out/


