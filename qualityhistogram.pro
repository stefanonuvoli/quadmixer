############################ CONFIGURATION ############################

#External libraries paths
EIGENPATH = /usr/include/eigen3
BOOSTPATH = /usr/include/boost
CGALPATH = /usr/include/CGAL
GUROBIPATH = /usr/include/gurobi
GLPATH = /usr/include/GL

#bin directory
DESTDIR = $$PWD/../bin

#######################################################################

# ----- We suggest to not modify anything under this line -----

#App config
TARGET = qualityhistogram

TEMPLATE        = app
QT             += core gui opengl widgets
CONFIG         += c++11
CONFIG         -= app_bundle
QMAKE_CXXFLAGS += -Wno-deprecated-declarations # gluQuadric gluSphere and gluCylinde are deprecated in macOS 10.9

#Debug/release optimization flags
CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next line if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}

#Final release optimization flag
FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}

# -------------- PROJECT FILES --------------

#Load libraries
include(libs/libs.pri)

SOURCES += \
    src/mainqualityhistogram.cpp
