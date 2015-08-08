cmake_minimum_required(VERSION 3.2)
project(nump)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tools/cmake/")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Build NUMP:

file(GLOB SOURCE_FILES "src/*.cpp" "src/*.h")
add_library(nump ${SOURCE_FILES})
set_target_properties(nump PROPERTIES LINKER_LANGUAGE CXX)


# Build tests:

set(NUMPTEST_SOURCES test/numptest.cpp)
add_executable(numptest ${NUMPTEST_SOURCES})
set_target_properties(numptest PROPERTIES LINKER_LANGUAGE CXX)


find_package(Cairo REQUIRED)
message("CID: ${CAIRO_INCLUDE_DIRS}")
include_directories(${CAIRO_INCLUDE_DIRS} REQUIRED)
#target_include_directories(numptest SYSTEM INTERFACE ${CAIRO_INCLUDE_DIRS} REQUIRED)
target_include_directories(numptest PRIVATE "src")


target_link_libraries(numptest
    nump
    ${CAIRO_LIBRARIES}
    )