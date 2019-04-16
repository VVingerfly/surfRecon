cmake_minimum_required(VERSION 3.0)

set(COMMON_HEADERS 
    ${HEADER_PATH}/mc/grid.h
    ${HEADER_PATH}/mc/mesh.h
	${HEADER_PATH}/mc/edge2vertexMap.h
)
set(COMMON_SRCFILES
    grid.cpp
	mcTable.cpp
)

set(MC_SRCS ${COMMON_HEADERS} ${COMMON_SRCFILES} 
    ${HEADER_PATH}/mc/marchingCubes.h 
	marchingCubes.cpp
)
add_library(mc STATIC ${MC_SRCS})
target_link_libraries(mc OpenMesh)
set_property(TARGET mc PROPERTY FOLDER "libs")

set(MCE_SRCS ${COMMON_HEADERS} ${COMMON_SRCFILES} 
    ${HEADER_PATH}/mc/marchingCubesExtended.h 
	marchingCubesExtended.cpp
)
add_library(mce STATIC ${MCE_SRCS})
target_link_libraries(mce OpenMesh)
set_property(TARGET mce PROPERTY FOLDER "libs")
