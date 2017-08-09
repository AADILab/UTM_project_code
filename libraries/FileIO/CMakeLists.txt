set( SRCS FileIn.cpp)
add_library( FileIO SHARED ${SRCS} )
target_link_libraries(FileIO)
