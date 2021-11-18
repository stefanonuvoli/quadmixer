############################ CONFIGURATION ############################

DEFINES += QUADRETOPOLOGY_DEBUG_SAVE_MESHES
#DEFINES += ASSERT_FOR_NUMBER_SIDES
#DEFINES += GUROBI_NON_VERBOSE
#DEFINES += NDEBUG


############################ LIBRARY PATHS ############################

#External libraries
GUROBI_PATH = /opt/gurobi903/linux64
CGAL_PATH = /usr/include/CGAL
GL_PATH = /usr/include/GL
EIGEN_PATH = /usr/include/eigen3
BOOST_PATH = /usr/include/boost

VCGLIB_PATH = $$PWD/libs/vcglib
LIBIGL_PATH = $$PWD/libs/libigl
QUADRETOPOLOGY_PATH = $$PWD/libs/quadretopology
