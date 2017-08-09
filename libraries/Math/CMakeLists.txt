set( SRCS easymath.cpp MatrixTypes.cpp)
add_library( Math SHARED ${SRCS} )
target_link_libraries(Math)
