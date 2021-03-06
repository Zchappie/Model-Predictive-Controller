#-------------------------------------------------
#
# Project created by QtCreator 2013-03-20T14:30:19
#
#-------------------------------------------------

QT       += network

QT       -= gui

TARGET = qmqtt
TEMPLATE = lib

DEFINES += QMQTT_LIBRARY

SOURCES += qmqtt_client.cpp \
    qmqtt_network.cpp \
    qmqtt_frame.cpp \
    qmqtt_client_p.cpp \
    qmqtt_message.cpp \
    qmqtt_router.cpp \
    qmqtt_will.cpp \
    qmqtt_routesubscription.cpp \
    qmqtt_routedmessage.cpp

HEADERS += qmqtt_client.h\
    qmqtt_global.h \
    qmqtt_network.h \
    qmqtt_frame.h \
    qmqtt_client_p.h \
    qmqtt_message.h \
    qmqtt_will.h \
    qmqtt_router.h \
    qmqtt.h \
    qmqtt_routesubscription.h \
    qmqtt_routedmessage.h

isEmpty(PREFIX) {
    contains(MEEGO_EDITION,harmattan) {
        PREFIX = /usr
    } else:unix:!symbian {
        maemo5 {
            PREFIX = /opt/usr
        } else {
            PREFIX = /usr/local
        }
    } else {
        PREFIX = $$[QT_INSTALL_PREFIX]
    }
}

headers.files = $$HEADERS
headers.path = $$PREFIX/include/qmqtt
target.path = $$PREFIX/lib
INSTALLS += headers target


OUTDIR = $$OUT_PWD/../out

#QMAKE_PRE_LINK = $(MKDIR)  $$OUT_PWD/../../out/
#QMAKE_POST_LINK += $(COPY) $$OUT_PWD/$$TEMPLATE$$TARGET* $$OUT_PWD/../../out/

OTHER_FILES += \
    qmqtt.pri
