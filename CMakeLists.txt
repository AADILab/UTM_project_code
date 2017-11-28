cmake_minimum_required(VERSION 2.8)
project(AbstractUTMSimulation)

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories(libraries)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(libraries)

set( LIBS_TO_LINK Domains FileIO Learning Math Multiagent Planning Simulation STL UTM yaml-cpp)

add_executable(AbstractUTMSimulation AbstractUTMSimulation.cpp)
target_link_libraries( AbstractUTMSimulation ${LIBS_TO_LINK} )

# Copy configuration and domain description files
# to the "installation" (currently build) directory
file(COPY config.yaml DESTINATION ${CMAKE_BINARY_DIR})
file(COPY Domains DESTINATION ${CMAKE_BINARY_DIR})

# Copy the postprocessing scripts
# to the "installation" directory
file(COPY scripts/pretty.py DESTINATION ${CMAKE_BINARY_DIR})
file(COPY scripts/replay_utm_log.m DESTINATION ${CMAKE_BINARY_DIR})

# Create the neural_nets directory
# If not present, a bunch of errors are printed:
# "Failed to open neural_nets/net_2_2.csv."
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/neural_nets)
