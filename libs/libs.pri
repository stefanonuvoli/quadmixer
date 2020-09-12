# -------------- INTERNAL LIBRARIES PATHS --------------

#Internal library (our work)
QUADBOOLEANPATH = $$PWD/quadboolean
PATTERNSPATH = $$PWD/patterns

#External libraries (included in the project because it works with these library versions only)
LIBIGLPATH = $$PWD/libigl
VCGLIBPATH = $$PWD/vcglib

# -------------- INTERNAL LIBRARIES INCLUDES --------------

#Quadboolean
include($$QUADBOOLEANPATH/quadboolean.pri)
INCLUDEPATH += $$QUADBOOLEANPATH

#Patterns
include($$PATTERNSPATH/patterns.pri)
INCLUDEPATH += $$PATTERNSPATH

#libigl
INCLUDEPATH += $$LIBIGLPATH/include/
QMAKE_CXXFLAGS += -isystem $$LIBIGLPATH/include/

#vcglib
INCLUDEPATH += $$VCGLIBPATH
DEFINES += NOCOMISO


# -------------- EXTERNAL LIBRARIES INCLUDES --------------

#eigen
INCLUDEPATH += $$EIGENPATH

#boost
INCLUDEPATH += $$BOOSTPATH

#glew
LIBS += -lGLEW -lglut -lGLU -lGL
#just for Linux
unix:!macx {
    DEFINES += GL_GLEXT_PROTOTYPES
    LIBS    += -lGLU
}
DEFINES += GLEW_STATIC
INCLUDEPATH += $$GLPATH

#gurobi
INCLUDEPATH += $$GUROBIPATH/include
LIBS += -L$$GUROBIPATH/lib -lgurobi_g++5.2 -lgurobi90
DEFINES += GUROBI_DEFINED

#Parallel computation
unix:!mac {
    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -fopenmp
}
macx{
    QMAKE_CXXFLAGS += -Xpreprocessor -fopenmp -lomp -I/usr/local/include
    QMAKE_LFLAGS += -lomp
    LIBS += -L /usr/local/lib /usr/local/lib/libomp.dylib
}
