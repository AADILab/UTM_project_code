set( SRCS SimNE.cpp)
add_library( Simulation STATIC ${SRCS} )
target_link_libraries(Simulation Domains FileIO Multiagent yaml-cpp)
