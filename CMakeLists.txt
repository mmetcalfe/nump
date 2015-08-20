cmake_minimum_required(VERSION 3.2)
project(nump)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tools/cmake/")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")

find_package(Cairo REQUIRED)
find_package(Armadillo REQUIRED)
include_directories(${CAIRO_INCLUDE_DIRS} REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS} REQUIRED)

# Build NUMP:

#add_library(nump src/nump.cpp)
##add_library(nump ${SOURCE_FILES} src/graph.h src/SearchTree.h)
#set_target_properties(nump PROPERTIES LINKER_LANGUAGE CXX)
#
#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS} REQUIRED)
#target_include_directories(nump PRIVATE "src")
#
#target_link_libraries(nump
#    ${ARMADILLO_LIBRARIES}
#    )

#file(GLOB SOURCE_FILES "src/*.h" "src/*.cpp" "test/*.h")
file(GLOB_RECURSE CPP_SOURCE_FILES "src/*.cpp" "test/*.cpp")


# Build tests:

#set(NUMPTEST_SOURCES test/numptest.cpp)
#add_executable(numptest ${NUMPTEST_SOURCES} ${SOURCE_FILES})
add_executable(numptest ${CPP_SOURCE_FILES})
set_target_properties(numptest PROPERTIES LINKER_LANGUAGE CXX)

#target_include_directories(numptest SYSTEM INTERFACE ${CAIRO_INCLUDE_DIRS} REQUIRED)
target_include_directories(numptest PRIVATE "src" "test")
#target_include_directories(numptest PRIVATE "test")


target_link_libraries(numptest
#    nump
    ${CAIRO_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    )