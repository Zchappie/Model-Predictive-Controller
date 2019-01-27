TEMPLATE = subdirs

CONFIG = ordered

SUBDIRS = \
    qmqtt \
    mpc \
    NlOptServer

mpc.depnds = qmqtt



