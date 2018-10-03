TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += ../src/main.cpp

INCLUDEPATH += ~/fuzzylite-6.0/fuzzylite/
LIBS += -L~/fuzzylite-6.0/fuzzylite/release/bin -lfuzzylite-static

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv
