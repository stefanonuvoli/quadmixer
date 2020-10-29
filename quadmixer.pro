############################ CONFIGURATION ############################

#External libraries paths
EIGENPATH = /usr/include/eigen3
BOOSTPATH = /usr/include/boost
CGALPATH = /usr/include/CGAL
GLPATH = /usr/include/GL
GUROBIPATH = /opt/gurobi903/linux64

#DEFINES += SAVE_MESHES_FOR_DEBUG

#######################################################################

# ----- We suggest to not modify anything under this line -----

#App config
TARGET = quadmixer

TEMPLATE        = app
QT             += core gui opengl widgets
CONFIG         += c++14
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

#Project files
SOURCES += \
    src/mainquadmixer.cpp \
    src/interface/quadmixerwindow.cpp \
    src/interface/glarea.cpp \
    src/interface/globjects/glpolywrap.cpp \
    src/interface/globjects/glquadlayoutwrap.cpp \
    src/interface/globjects/glchartsideswrap.cpp \
    src/interface/globjects/gldrawtext.cpp \
    src/interface/globjects/glverticeswrap.cpp \
    src/interface/globjects/glsegmentswrap.cpp

HEADERS += \
    src/interface/quadmixerwindow.h \
    src/interface/glarea.h \
    src/interface/globjects/glpolywrap.h \
    src/interface/globjects/glquadlayoutwrap.h \
    src/interface/globjects/glchartsideswrap.h \
    src/interface/globjects/gldrawtext.h \
    src/interface/globjects/glverticeswrap.h \
    src/interface/globjects/glsegmentswrap.h

FORMS += \
    src/interface/quadmixerwindow.ui


HEADERS += \
    $$VCGLIBPATH/wrap/ply/plylib.h \
    $$VCGLIBPATH/wrap/gui/trackmode.h \
    $$VCGLIBPATH/wrap/gui/trackball.h
SOURCES += \
    $$VCGLIBPATH/wrap/ply/plylib.cpp \
    $$VCGLIBPATH/wrap/gui/trackmode.cpp \
    $$VCGLIBPATH/wrap/gui/trackball.cpp
