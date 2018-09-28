cmake_minimum_required(VERSION 3.10)
project(Parse_OpenMesh_FCL)

set(CMAKE_CXX_STANDARD 11)
set(LIB_OPEN_MESH_DIR ../../../../../../usr/local/lib)

add_library(parse_om_fcl SHARED get_OpenMesh_data.cpp get_OpenMesh_data.h)

find_library(LIB_OPEN_MESH_BIN OpenMeshCore OpenMeshTools PATHS ${LIB_OPEN_MESH_DIR} NO_DEFAULT_PATH)
find_library(FCL_LIB libfcl.so.0.5 PATHS /usr/local/lib NO_DEFAULT_PATH)

target_link_libraries(parse_om_fcl ${FCL_LIB})
target_link_libraries(parse_om_fcl ${LIB_OPEN_MESH_BIN})