set( SRCS IDomainStateful.cpp)
add_library( Domains STATIC ${SRCS} )
target_link_libraries(Domains Simulation yaml-cpp)

add_subdirectory(UTM)
