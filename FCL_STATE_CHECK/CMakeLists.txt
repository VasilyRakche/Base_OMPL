cmake_minimum_required(VERSION 3.10)
project(FCL_STATE_CHECK)
include_directories("../../omplapp/include/")

set(CMAKE_CXX_STANDARD 11)
set(LIB_OMPL_DIR ../../omplapp/lib)

add_library(fcl_state_check SHARED FCL_StateInspection.cpp FCL_StateInspection.h)
find_library(OMPL_LIB ompl PATHS ${LIB_OMPL_DIR})
target_link_libraries(fcl_state_check ${OMPL_LIB})

find_library(FCL_LIB libfcl.so.0.5 PATHS /usr/local/lib NO_DEFAULT_PATH)
target_link_libraries(fcl_state_check ${FCL_LIB})