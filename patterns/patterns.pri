LPSOLVER_PATH = $$PWD/ktmethod/lp_solve
LPSOLVER_PATH0 = $$PWD/ktmethod/lp_solve/bfp/
LPSOLVER_PATH1 = $$PWD/ktmethod/lp_solve/colamd/

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/ktmethod/lp_solve/release/ -llpsolve55
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/ktmethod/lp_solve/debug/ -llpsolve55
else:unix: LIBS += -L$$PWD/ktmethod/lp_solve/ -llpsolve55

DEPENDPATH += $$PWD/ktmethod/lp_solve

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/ktmethod/lp_solve/release/liblpsolve55.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/ktmethod/lp_solve/debug/liblpsolve55.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/ktmethod/lp_solve/release/lpsolve55.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/ktmethod/lp_solve/debug/lpsolve55.lib
else:unix: PRE_TARGETDEPS += $$PWD/ktmethod/lp_solve/liblpsolve55.a

LIBS += -ldl -lm -lmpfr -llpsolve55
INCLUDEPATH += $$LPSOLVER_PATH $$EIGEN_PATH

QMAKE_CXXFLAGS += -std=c++11 -fpermissive

SOURCES += \
    $$PWD/myutils.cpp \
    $$PWD/ktmethod/patchgen/get_boundary_geometry.cpp \
    $$PWD/ktmethod/patchgen/get_default_parameter.cpp \
    $$PWD/ktmethod/patchgen/get_param_str.cpp \
    $$PWD/ktmethod/patchgen/extradefinition.cpp

HEADERS += \
    $$PWD/laplacianreconstruction.h \
    $$PWD/meshtypes.h \
    $$PWD/myutils.h \
    $$PWD/patchg.h \
    $$PWD/ktmethod/lp_solve/lp_lib.h \
    $$PWD/ktmethod/patchgen/Permutation.h \
    $$PWD/ktmethod/patchgen/Pattern_all.h \
    $$PWD/ktmethod/patchgen/Pattern_6_3.h \
    $$PWD/ktmethod/patchgen/Pattern_6_2.h \
    $$PWD/ktmethod/patchgen/Pattern_6_1.h \
    $$PWD/ktmethod/patchgen/Pattern_6_0.h \
    $$PWD/ktmethod/patchgen/Pattern_5_4.h \
    $$PWD/ktmethod/patchgen/Pattern_5_3.h \
    $$PWD/ktmethod/patchgen/Pattern_5_2.h \
    $$PWD/ktmethod/patchgen/Pattern_5_1.h \
    $$PWD/ktmethod/patchgen/Pattern_5_0.h \
    $$PWD/ktmethod/patchgen/Pattern_4_4.h \
    $$PWD/ktmethod/patchgen/Pattern_4_3.h \
    $$PWD/ktmethod/patchgen/Pattern_4_2.h \
    $$PWD/ktmethod/patchgen/Pattern_4_1.h \
    $$PWD/ktmethod/patchgen/Pattern_4_0.h \
    $$PWD/ktmethod/patchgen/Pattern_3_3.h \
    $$PWD/ktmethod/patchgen/Pattern_3_2.h \
    $$PWD/ktmethod/patchgen/Pattern_3_1.h \
    $$PWD/ktmethod/patchgen/Pattern_3_0.h \
    $$PWD/ktmethod/patchgen/Pattern_2_1.h \
    $$PWD/ktmethod/patchgen/Pattern_2_0.h \
    $$PWD/ktmethod/patchgen/Pattern.h \
    $$PWD/ktmethod/patchgen/PatchParam.h \
    $$PWD/ktmethod/patchgen/ILP.h \
    $$PWD/ktmethod/patchgen/generate_subtopology.h \
    $$PWD/ktmethod/patchgen/decl.h \
    $$PWD/ktmethod/patchgen/edgeloop.h
