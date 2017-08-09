cmake_minimum_required(VERSION 2.8)
project(AbstractUTMSimulation)

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories(libraries)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
add_subdirectory(libraries)

set( LIBS_TO_LINK Domains FileIO Learning Math Multiagent Planning Simulation STL UTM yaml-cpp)

add_executable(AbstractUTMSimulation AbstractUTMSimulation.cpp)
target_link_libraries( AbstractUTMSimulation ${LIBS_TO_LINK} )
