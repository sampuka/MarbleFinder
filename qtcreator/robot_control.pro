TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += ../src/main.cpp \
    ../src/LaserScanner.cpp \
    ../src/FuzzyBugController.cpp \
    ../src/Brushfire.cpp \
    ../src/Controller.cpp \
    ../src/WorldMapper.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv


unix:!macx: LIBS += /home/sampuka/fuzzylite-6.0/release/bin/libfuzzylite-static.a

INCLUDEPATH += /home/sampuka/fuzzylite-6.0/fuzzylite


INCLUDEPATH += ../include/
HEADERS += \
    ../include/LaserScanner.hpp \
    ../include/FuzzyBugController.hpp \
    ../include/Brushfire.hpp \
    ../include/Controller.hpp \
    ../include/WorldMapper.hpp

DISTFILES += \
    ../123.fll

