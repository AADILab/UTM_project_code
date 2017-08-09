set( SRCS GridGraph.cpp LinkGraph.cpp)
add_library( Planning SHARED ${SRCS} )
target_link_libraries(Planning FileIO Math STL)
