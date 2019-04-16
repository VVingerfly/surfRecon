cmake_minimum_required(VERSION 3.0)

set(SRCS main.cpp)
add_executable(test_mc ${SRCS})
target_link_libraries(test_mc OpenMesh mc mce)
set_property(TARGET test_mc PROPERTY FOLDER "tests")
