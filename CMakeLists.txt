cmake_minimum_required(VERSION 2.8.11)
project(v_repExtOctomap)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

find_package(Boost REQUIRED COMPONENTS system regex)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
find_package(VREP 3.5.0 REQUIRED)
find_package(Octomap REQUIRED)

include_directories(${VREP_INCLUDE})
include_directories(${CMAKE_BINARY_DIR}/generated)
include_directories(${CMAKE_SOURCE_DIR}/external)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIR})

vrep_generate_stubs(${CMAKE_BINARY_DIR}/generated XML_FILE ${CMAKE_SOURCE_DIR}/callbacks.xml LUA_FILE ${CMAKE_SOURCE_DIR}/simExtOctomap.lua)

set(SOURCES
    plugin.cpp
    ${CMAKE_SOURCE_DIR}/external/v_repPlusPlus/Plugin.cpp
    ${CMAKE_BINARY_DIR}/generated/stubs.cpp
    ${VREP_EXPORTED_SOURCES}
)
add_library(v_repExtOctomap SHARED ${SOURCES})
target_link_libraries(v_repExtOctomap ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${VREP_LIBRARIES})

install(TARGETS v_repExtOctomap DESTINATION ${VREP_ROOT})

