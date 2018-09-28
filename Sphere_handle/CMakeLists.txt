cmake_minimum_required(VERSION 3.10)
project(Sphere_handle_)

set(CMAKE_CXX_STANDARD 11)

add_library(sphere_handle SHARED WorkspaceSphereExplorer.cpp WorkspaceSphereExplorer.h WorkspaceSphere.cpp WorkspaceSphere.h WorkspaceSphereList.h Sphere_aproximation.cpp Sphere_aproximation.h)
#Find libraries
find_library(FCL_LIB libfcl.so.0.5 PATHS /usr/local/lib NO_DEFAULT_PATH)
find_library(FCL_STATE_CHECK fcl_state_check PATHS ../cmake-build-debug/FCL_STATE_CHECK)
find_library(LIB_OPEN_MESH_BIN OpenMeshCore OpenMeshTools PATHS ../../../../../usr/local/lib NO_DEFAULT_PATH)
#End find libraries
#Link libraries
target_link_libraries(sphere_handle ${FCL_STATE_CHECK} ${FCL_LIB} ${LIB_OPEN_MESH_BIN})