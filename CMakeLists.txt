cmake_minimum_required(VERSION 2.8)
project(AbstractUTMSimulation)

find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(Threads)

include_directories( ${Boost_INCLUDE_DIRS} )
include_directories(libraries)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(libraries)

set( LIBS_TO_LINK Domains
                  FileIO
                  Learning
                  Math
                  Multiagent
                  Planning
                  Simulation
                  STL
                  UTM
                  yaml-cpp
                  ${CMAKE_THREAD_LIBS_INIT})

add_executable(AbstractUTMSimulation AbstractUTMSimulation.cpp threadpool.cpp)
target_link_libraries( AbstractUTMSimulation ${LIBS_TO_LINK} )

#add_executable(testNNLoad testNNLoad.cpp)
#target_link_libraries( testNNLoad ${LIBS_TO_LINK} )

#add_executable(AbstractUTMTesting AbstractUTMTesting.cpp)
#target_link_libraries( AbstractUTMTesting ${LIBS_TO_LINK} )

# Copy configuration and domain description files
# to the "installation" (currently build) directory
file(COPY config.yaml DESTINATION ${CMAKE_BINARY_DIR})
file(COPY Domains DESTINATION ${CMAKE_BINARY_DIR})

# Symlink the postprocessing scripts
# to the "installation" directory
# This will only work on systems that support symlinks,
# i.e, unix-like systems.
ADD_CUSTOM_TARGET(symlink_scripts ALL
                                  COMMAND ${CMAKE_COMMAND} -E
                                  create_symlink
                                  ${CMAKE_SOURCE_DIR}/scripts/summarize.py
                                  ${CMAKE_BINARY_DIR}/summarize.py)

# Create the neural_nets directory
# If not present, a bunch of errors are printed:
# "Failed to open neural_nets/net_2_2.csv."
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/neural_nets)
