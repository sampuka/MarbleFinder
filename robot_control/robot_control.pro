TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    laserscanner.cpp \
    fuzzybugcontroller.cpp \
    visioncamera.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv


unix:!macx: LIBS += /home/thor/fuzzylite-6.0/release/bin/libfuzzylite-static.a

INCLUDEPATH +=/home/thor/fuzzylite-6.0/fuzzylite


HEADERS += \
    laserscanner.h \
    fuzzybugcontroller.h \
    visioncamera.h

DISTFILES += \
    fuzzybugcontroller.fll \
    123.fll

