############################ CONFIGURATION ############################

#DEFINES += QUADRETOPOLOGY_DEBUG_SAVE_MESHES
#DEFINES += ASSERT_FOR_NUMBER_SIDES
#DEFINES += GUROBI_NON_VERBOSE
#DEFINES += NDEBUG


############################ LIBRARY PATHS ############################

#External libraries
VCGLIB_PATH         = $$PWD/libs/vcglib
LIBIGL_PATH         = $$PWD/libs/libigl
QUADRETOPOLOGY_PATH = $$PWD/libs/quadretopology

GL_PATH             = /usr/include/GL
EIGEN_PATH          = /usr/include/eigen3
BOOST_PATH          = /usr/include/boost
CGAL_PATH           = /usr/include
GUROBI_PATH         = /opt/gurobi950/linux64
GUROBI_COMPILER     = gurobi_g++5.2
GUROBI_LIB          = gurobi95
