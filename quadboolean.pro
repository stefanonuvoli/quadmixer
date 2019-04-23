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

LIBIGLPATH = /usr/include/libigl
EIGENPATH = /usr/include/eigen3
BOOSTPATH = /usr/include/boost
VCGLIBPATH = /usr/include/vcglib
CGALPATH = /usr/include/CGAL
GUROBIPATH = /usr/include/gurobi
MOSEKPATH = /usr/include/mosek/8/tools/platform/linux64x86
PATTERNSPATH = $$PWD/patterns

#Quad booleans
SOURCES += \
    quadboolean.cpp \
    quadpatchtracer.tpp \
    quadpatches.cpp \
    quadilp.cpp \
    quadbooleanwindow.cpp \
    quadconvert.tpp \
    quadbooleaninterface.cpp \
    quadutils.cpp \
    glpolywrap.tpp \
    main.cpp \
    quadcharts.cpp \
    glquadlayoutwrap.tpp \
    glchartsideswrap.tpp \
    glarea.cpp \
    gldrawtext.cpp \
    quadpatterns.cpp \
    quadquadmapping.cpp

HEADERS += \
    quadboolean.h \
    quadpatchtracer.h \
    quadpatches.h \
    quadilp.h \
    quadbooleanwindow.h \
    quadbooleaninterface.h \
    quadcommontypes.h \
    quadconvert.h \
    quadutils.h \
    glpolywrap.h \
    quadcharts.h \
    glquadlayoutwrap.h \
    glchartsideswrap.h \
    glarea.h \
    gldrawtext.h \
    quadpatterns.h \
    quadquadmapping.h

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
INCLUDEPATH += /usr/include/GL

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

#mosek
MODULES += MOSEK
INCLUDEPATH += $$MOSEKPATH/h
LIBS += -L$$MOSEKPATH/bin -lfusion64 -lmosek64
DEFINES += MOSEK_DEFINED


message(Included modules: $$MODULES)


##QUADRANGULATION
#SOURCES += \
#    charts.cpp \
#    patterns.cpp \
#    quadrangulation.cpp \
#    testquadrangulation.cpp \
#    maintestquadrangulation.cpp
#HEADERS += \
#    charts.h \
#    patterns.h \
#    quadrangulation.h \
#    testquadrangulation.h



## Patch decomposition
#HEADERS += \
#    $$VCGLIBPATH/wrap/ply/plylib.h \
#    $$VCGLIBPATH/wrap/gui/trackmode.h \
#    $$VCGLIBPATH/wrap/gui/trackball.h \
#    mainwindow.h \
#    glarea.h \
#    patchdecomposition.h
#SOURCES += \
#    $$VCGLIBPATH/wrap/ply/plylib.cpp \
#    $$VCGLIBPATH/wrap/gui/trackmode.cpp \
#    $$VCGLIBPATH/wrap/gui/trackball.cpp \
#    mainwindow.cpp \
#    glarea.cpp \
#    mainpatchdecomposition.cpp \
#    patchdecomposition.cpp
#FORMS += mainwindow.ui
#LIBS += -lGLEW



#DEFINES += MULTI_LABEL_OPTIMIZATION_INCLUDED
#INCLUDEPATH += $$PWD/$$MULTILABELOPTIMIZATIONPATH
#SOURCES += \
#    $$MULTILABELOPTIMIZATIONPATH/GCoptimization.cpp \
#    $$MULTILABELOPTIMIZATIONPATH/graph.cpp \
#    $$MULTILABELOPTIMIZATIONPATH/LinkedBlockList.cpp \
#    $$MULTILABELOPTIMIZATIONPATH/maxflow.cpp \
#    $$MULTILABELOPTIMIZATIONPATH/example.cpp



#CINOLIBPATH = /usr/include/cinolib
#MULTILABELOPTIMIZATIONPATH = $$PWD/MultiLabelOptimization
#DIRECTIONALPATH = /usr/include/Directional

##cg3
#CONFIG += CG3_CORE CG3_MESHES
#include(../cg3lib/cg3.pri)
#DEFINES += CG3_IGNORE_TYPESAFE_SERIALIZATION_CHECK

##Directional
#INCLUDEPATH += $$DIRECTIONALPATH/include/

##cinolib
#DEFINES += CINOLIB_DEFINED
#MODULES += CINOLIB

#DEFINES += CINOLIB_USES_OPENGL
#DEFINES += CINOLIB_USES_QT

#macx{
#    QMAKE_CXXFLAGS   = -Wno-c++11-extensions
#}
#INCLUDEPATH     += $$CINOLIBPATH/include/ #-> link to cinolib
#QMAKE_CXXFLAGS += -isystem $$CINOLIBPATH #-> link to cinolib

## just for Linux
#unix:!macx {
#    DEFINES += GL_GLEXT_PROTOTYPES
#    LIBS    += -lGLU
#}

## Compile glew
#DEFINES += GLEW_STATIC
#INCLUDEPATH += /usr/include/GL
