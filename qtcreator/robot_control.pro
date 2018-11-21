TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

#QMAKE_CXXFLAGS += -fsanitize=thread
#QMAKE_LFLAGS += -fsanitize=thread

SOURCES += ../src/main.cpp \
    ../src/LaserScanner.cpp \
    ../src/FuzzyBugController.cpp \
    ../src/Brushfire.cpp \
    ../src/Controller.cpp \
    ../src/WorldMapper.cpp \
    ../src/astar.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

unix:!macx: LIBS += /home/thor/fuzzylite-6.0/release/bin/libfuzzylite-static.a

INCLUDEPATH += /home/thor/fuzzylite-6.0/fuzzylite

INCLUDEPATH += ../include/

HEADERS += \
    ../include/LaserScanner.hpp \
    ../include/FuzzyBugController.hpp \
    ../include/Brushfire.hpp \
    ../include/Controller.hpp \
    ../include/WorldMapper.hpp \
    ../include/astar.hpp

DISTFILES += \
    ../123.fll \
    ../avoider.fll
