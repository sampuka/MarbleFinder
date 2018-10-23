TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += ../src/main.cpp \
    ../src/laserscanner.cpp \
    ../src/fuzzybugcontroller.cpp \
    ../src/visioncamera.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv


unix:!macx: LIBS += /home/thor/fuzzylite-6.0/release/bin/libfuzzylite-static.a

INCLUDEPATH +=/home/thor/fuzzylite-6.0/fuzzylite


INCLUDEPATH += ../include/
HEADERS += \
    ../include/laserscanner.h \
    ../include/fuzzybugcontroller.h \
    ../include/visioncamera.h

DISTFILES += \
    fuzzybugcontroller.fll \
    123.fll

