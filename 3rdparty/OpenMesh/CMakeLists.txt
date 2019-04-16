cmake_minimum_required(VERSION 3.0)


# OpenMesh library
set(OPENMESH_SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/OpenMesh/Core")
message(STATUS "OPENMESH_SOURCE_DIR = " ${OPENMESH_SOURCE_DIR})
file(GLOB_RECURSE OPENMESH_HEADERS "${OPENMESH_SOURCE_DIR}/*.hh" "${OPENMESH_SOURCE_DIR}/*.h")
file(GLOB_RECURSE OPENMESH_SOURCES "${OPENMESH_SOURCE_DIR}/*.cc")
add_library(OpenMesh STATIC ${OPENMESH_HEADERS} ${OPENMESH_SOURCES})
#add_definitions(-DOM_STATIC_BUILD)
target_compile_definitions(OpenMesh PUBLIC OM_STATIC_BUILD)

set_property(TARGET OpenMesh PROPERTY FOLDER "libs")
