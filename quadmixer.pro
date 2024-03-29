############################ TARGET AND FLAGS ############################

#App config
TARGET = quadmixer
TEMPLATE = app
CONFIG += c++14
CONFIG += qt
CONFIG -= app_bundle
QT += core gui opengl widgets

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

macx {
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.13
    QMAKE_MAC_SDK = macosx10.13
}


############################ LIBRARIES ############################

#Setting library paths and configuration
include(configuration.pri)

#Quad retopology
include($$QUADRETOPOLOGY_PATH/quadretopology.pri)

#Libigl
INCLUDEPATH += $$LIBIGL_PATH/include/
QMAKE_CXXFLAGS += -isystem $$LIBIGL_PATH/include/

#Vcglib
INCLUDEPATH += $$VCGLIB_PATH
DEFINES += NOCOMISO

#Cgal
#DEFINES += CGAL_EIGEN3_ENABLED
#LIBS += -lCGAL -lCGAL_Core
INCLUDEPATH += $$CGAL_PATH
LIBS += -lmpfr -lgmp -frounding-math
LIBS += -lboost_system -DBOOST_LOG_DYN_LINK -lboost_log -lboost_thread -lpthread

#Eigen
INCLUDEPATH += $$EIGEN_PATH

#Boost
INCLUDEPATH += $$BOOST_PATH

#glew
LIBS += -lGLEW -lglut -lGLU -lGL
#Just for Linux
unix:!macx {
    DEFINES += GL_GLEXT_PROTOTYPES
    LIBS    += -lGLU
}
DEFINES += GLEW_STATIC
INCLUDEPATH += $$GL_PATH

#Gurobi
INCLUDEPATH += $$GUROBI_PATH/include
LIBS += -L$$GUROBI_PATH/lib -l$$GUROBI_COMPILER -l$$GUROBI_LIB
DEFINES += GUROBI_DEFINED

LIBS += -lblosc -ltbb -lHalf -lboost_thread -lboost_system -lboost_iostreams

#Parallel computation
unix:!mac {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -fopenmp
}
#macx{
#    QMAKE_CXXFLAGS += -Xpreprocessor -fopenmp -lomp -I/usr/local/include
#    QMAKE_LFLAGS += -lomp
#    LIBS += -L /usr/local/lib /usr/local/lib/libomp.dylib
#}

win32{
    DEFINES += NOMINMAX # Awful problem with windows..
    DEFINES *= _USE_MATH_DEFINES
    DEFINES *= _SCL_SECURE_NO_DEPRECATE
    QMAKE_CXXFLAGS *= /bigobj
}

############################ PROJECT FILES ############################

INCLUDEPATH += $$PWD/src/

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
    src/interface/globjects/glsegmentswrap.cpp \
    src/quadmixer/includes/quadfeasibility.cpp \
    src/quadmixer/includes/quadlayoutdata.cpp \
    src/quadmixer/includes/quadlibiglbooleaninterface.cpp \
    src/quadmixer/includes/quadpatchtracer.cpp \
    src/quadmixer/includes/quadpreserved.cpp \
    src/quadmixer/includes/quadbooleansteps.cpp \
    src/quadmixer/quadboolean.cpp

HEADERS += \
    src/interface/quadmixerwindow.h \
    src/interface/glarea.h \
    src/interface/globjects/glpolywrap.h \
    src/interface/globjects/glquadlayoutwrap.h \
    src/interface/globjects/glchartsideswrap.h \
    src/interface/globjects/gldrawtext.h \
    src/interface/globjects/glverticeswrap.h \
    src/interface/globjects/glsegmentswrap.h \
    src/quadmixer/defaultmeshtypes.h \
    src/quadmixer/includes/envelope_generator.h \
    src/quadmixer/includes/quadbooleancommon.h \
    src/quadmixer/includes/quadfeasibility.h \
    src/quadmixer/includes/quadlayoutdata.h \
    src/quadmixer/includes/quadlibiglbooleaninterface.h \
    src/quadmixer/includes/quadpatchtracer.h \
    src/quadmixer/includes/quadpreserved.h \
    src/quadmixer/includes/quadbooleansteps.h \
    src/quadmixer/quadboolean.h

FORMS += \
    src/interface/quadmixerwindow.ui

HEADERS += \
    $$VCGLIB_PATH/wrap/gui/trackmode.h \
    $$VCGLIB_PATH/wrap/gui/trackball.h
SOURCES += \
    $$VCGLIB_PATH/wrap/gui/trackmode.cpp \
    $$VCGLIB_PATH/wrap/gui/trackball.cpp

#Vcg ply (needed to save ply files)
HEADERS += \
    $$VCGLIB_PATH/wrap/ply/plylib.h
SOURCES += \
    $$VCGLIB_PATH/wrap/ply/plylib.cpp
