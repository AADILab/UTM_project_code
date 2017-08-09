set( SRCS MultiagentNE.cpp)
add_library( Multiagent STATIC ${SRCS} )
target_link_libraries(Multiagent Domains Learning)
