cmake_minimum_required(VERSION 3.0)

project(SurfRecon)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_VERBOSE_MAKEFILE OFF)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Additional compiler flags
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	message("Clang compiler found.")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
	message("AppleClang compiler found.")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	message("GNU compiler found.")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("MSVC compiler found.")
    add_definitions(/DUSE_MSVC)
    add_definitions(/D_USE_MATH_DEFINES)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(HEADER_PATH ${PROJECT_SOURCE_DIR}/include)

SET(DATA_PATH ${PROJECT_SOURCE_DIR}/data)

include_directories(${HEADER_PATH})
include_directories(${HEADER_PATH}/mc)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty)

link_directories(${LIBRARY_OUTPUT_PATH})
link_directories(${LIBRARY_OUTPUT_PATH}/Debug)


# Detect Eigen3 in the external folder
if(EXISTS ${PROJECT_SOURCE_DIR}/include/eigen3/Eigen/Dense)
	message("Found user-provided Eigen.")
	set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/eigen3")
	include_directories(${EIGEN3_INCLUDE_DIR})
else()
	find_package(Eigen3 REQUIRED)
	if(EIGEN3_FOUND)
		message("Found system-installed Eigen")
		include_directories(${EIGEN3_INCLUDE_DIR})
	endif()
endif()

add_definitions("-DDATAPATH=\"${DATA_PATH}/\"")

add_subdirectory(3rdparty)
add_subdirectory(src)

