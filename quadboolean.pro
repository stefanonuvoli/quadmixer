# Executable name
TARGET = quadboolean

TEMPLATE        = app
QT             += core gui opengl widgets
CONFIG         += c++11
CONFIG         -= app_bundle
QMAKE_CXXFLAGS += -Wno-deprecated-declarations # gluQuadric gluSphere and gluCylinde are deprecated in macOS 10.9

CONFIG(debug, debug|release){
    DEFINES += DEBUG
}
CONFIG(release, debug|release){
    DEFINES -= DEBUG
    #just uncomment next lines if you want to ignore asserts and got a more optimized binary
    CONFIG += FINAL_RELEASE
}
FINAL_RELEASE {
    unix:!macx{
        QMAKE_CXXFLAGS_RELEASE -= -g -O2
        QMAKE_CXXFLAGS += -O3 -DNDEBUG
    }
}
FINAL_RELEASE {
    message(Final Release!)
}

EIGENPATH = /usr/include/eigen3
BOOSTPATH = /usr/include/boost
CGALPATH = /usr/include/CGAL
GUROBIPATH = /usr/include/gurobi
GLPATH = /usr/include/GL

LIBIGLPATH = $$PWD/libigl
VCGLIBPATH = $$PWD/vcglib
PATTERNSPATH = $$PWD/patterns

#Quad booleans
SOURCES += \
    main.cpp \
    quadboolean.tpp \
    quadbooleanwindow.cpp \
    glarea.cpp \
    quadboolean/quadpatchtracer.tpp \
    quadboolean/quadilp.cpp \
    quadboolean/quadconvert.tpp \
    quadboolean/quadcharts.tpp \
    quadboolean/quadpatterns.cpp \
    quadboolean/quadquadmapping.cpp \
    quadboolean/quadlayoutdata.tpp \
    quadboolean/quadpreserved.tpp \
    quadboolean/quadlibiglbooleaninterface.cpp \
    quadboolean/quadbooleansteps.tpp \
    globjects/glpolywrap.tpp \
    globjects/glquadlayoutwrap.tpp \
    globjects/glchartsideswrap.tpp \
    globjects/gldrawtext.cpp \
    quadboolean/quadutils.tpp

HEADERS += \
    quadboolean.h \
    quadbooleanwindow.h \
    glarea.h \
    quadboolean/quadpatchtracer.h \
    quadboolean/quadilp.h \
    quadboolean/quadconvert.h \
    quadboolean/quadutils.h \
    quadboolean/quadcharts.h \
    quadboolean/quadpatterns.h \
    quadboolean/quadquadmapping.h \
    quadboolean/quadlayoutdata.h \
    quadboolean/quadpreserved.h \
    quadboolean/quadlibiglbooleaninterface.h \
    quadboolean/patch_decomposer.h \
    quadboolean/field_tracer.h \
    quadboolean/quadbooleansteps.h \
    quadboolean/quadbooleanoperation.h \
    globjects/glpolywrap.h \
    globjects/glquadlayoutwrap.h \
    globjects/glchartsideswrap.h \
    globjects/gldrawtext.h \
    meshtypes.h

FORMS += \
    quadbooleanwindow.ui

LIBS += -lGLEW -lglut
#just for Linux
unix:!macx {
    DEFINES += GL_GLEXT_PROTOTYPES
    LIBS    += -lGLU
}

#Compile glew
DEFINES += GLEW_STATIC
INCLUDEPATH += $$GLPATH

#eigen
MODULES += EIGEN
INCLUDEPATH += $$EIGENPATH

#boost
MODULES += BOOST
INCLUDEPATH += $$BOOSTPATH

#cgal
#DEFINES += CGAL_EIGEN3_ENABLED
MODULES += CGAL
LIBS += -lmpfr -lgmp -lCGAL -frounding-math -lCGAL_Core
LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread

#libigl
MODULES += LIBIGL
INCLUDEPATH += $$LIBIGLPATH/include/
QMAKE_CXXFLAGS += -isystem $$LIBIGLPATH/include/
#vcglib
MODULES += VCGLIB
DEFINES += VCGLIB_DEFINED
INCLUDEPATH += $$VCGLIBPATH
DEFINES += NOCOMISO
HEADERS += \
    $$VCGLIBPATH/wrap/ply/plylib.h \
    $$VCGLIBPATH/wrap/gui/trackmode.h \
    $$VCGLIBPATH/wrap/gui/trackball.h
SOURCES += \
    $$VCGLIBPATH/wrap/ply/plylib.cpp \
    $$VCGLIBPATH/wrap/gui/trackmode.cpp \
    $$VCGLIBPATH/wrap/gui/trackball.cpp

#patterns
include($$PATTERNSPATH/patterns.pri)

#gurobi
MODULES += GUROBI
INCLUDEPATH += $$GUROBIPATH/include
LIBS += -L$$GUROBIPATH/lib -lgurobi_g++5.2 -lgurobi81
DEFINES += GUROBI_DEFINED

message(Included modules: $$MODULES)
