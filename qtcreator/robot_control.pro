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
    ../src/astar.cpp \
<<<<<<< HEAD
    ../src/Marblelocator.cpp
=======
    ../src/dijkstra.cpp
>>>>>>> 0c955524c10b720d94d299b9901106f2b61e81fb

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
    ../include/WorldMapper.hpp \
    ../include/astar.hpp \
<<<<<<< HEAD
    ../include/Marblelocator.hpp
=======
    ../include/dijkstra.hpp
>>>>>>> 0c955524c10b720d94d299b9901106f2b61e81fb

DISTFILES += \
    ../123.fll \
    ../avoider.fll
