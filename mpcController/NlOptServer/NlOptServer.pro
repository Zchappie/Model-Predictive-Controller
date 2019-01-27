QT += core network
QT -= gui

QMAKE_CXXFLAGS += -std=c++11

TARGET = NlOptServer
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    nloptnetwork.cpp \
    nloptthread.cpp \
    ../mpc/constraint.cpp \
    ../mpc/cell.cpp \
    ../mpc/vectorhelper.cpp \
    ../mpc/model.cpp \
    ../mpc/CostFunction.cpp

HEADERS += \
    nloptnetwork.h \
    nloptthread.h \
    ../mpc/constraint.h \
    ../mpc/cell.h \
    ../mpc/vectorhelper.h \
    ../mpc/model.h \
    ../mpc/CostFunction.h

INCLUDEPATH += "$$PWD/../nlopt"

win32:CONFIG(release, debug|release): LIBS += -L"$$PWD/../nlopt/libs/win" -lnlopt-0
else:win32:CONFIG(debug, debug|release): LIBS += -L"$$PWD/../nlopt/libs/win" -lnlopt-0
else:unix: LIBS += -L$$_PRO_FILE_PWD_/../nlopt/libs -lnlopt
