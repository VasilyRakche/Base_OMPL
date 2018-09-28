cmake_minimum_required(VERSION 3.10)
project(Base_OMPL)
include_directories("../../omplapp/include/")

set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_STANDARD 11)
set(LIB_OMPL_DIR ../../omplapp/lib)
#Adding subdirectories for project
add_subdirectory(Parse_OpenMesh>FCL)
add_subdirectory(FCL_STATE_CHECK)
add_subdirectory(Sphere_handle)
#End adding subdirectories for project

add_executable(Base_OMPL main.cpp isValid_class.h )
#Find standard libraries OMPL,OpenMesh,FCL
find_library(OMPL_LIB ompl PATHS ${LIB_OMPL_DIR})
find_library(LIB_OPEN_MESH_BIN OpenMeshCore OpenMeshTools PATHS ${LIB_OPEN_MESH_DIR} NO_DEFAULT_PATH)
find_library(FCL_LIB libfcl.so.0.5 PATHS /usr/local/lib NO_DEFAULT_PATH)
#Find my libraries
find_library(PARSE_OM_FCL parse_om_fcl PATHS cmake-build-debug/Parse_OpenMesh>FCL)
find_library(FCL_STATE_CHECK fcl_state_check PATHS cmake-build-debug/FCL_STATE_CHECK)
find_library(SPHERE_HANDLE_DIR sphere_handle cmake-build-debug/Sphere_handle)

#End find libraries
message(STATUS ${SPHERE_HANDLE_DIR})
#Link libraries
target_link_libraries(Base_OMPL ${OMPL_LIB} ${PARSE_OM_FCL} ${FCL_LIB} ${LIB_OPEN_MESH_BIN} ${FCL_STATE_CHECK} ${SPHERE_HANDLE_DIR})

