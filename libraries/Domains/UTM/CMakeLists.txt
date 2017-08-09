set( SRCS IAgentBody.cpp Link.cpp Sector.cpp UAV.cpp UTMDomainAbstract.cpp)
add_library( UTM SHARED ${SRCS} )
target_link_libraries(UTM Domains FileIO Learning Math Planning Simulation STL yaml-cpp)
