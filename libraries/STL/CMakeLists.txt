set( SRCS easystl.cpp)
add_library( STL SHARED ${SRCS} )
target_link_libraries(STL)
