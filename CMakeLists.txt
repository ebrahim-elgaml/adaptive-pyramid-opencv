cmake_minimum_required(VERSION 2.8)
project( Adaptive-pyramid )
find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Adaptive-pyramid Main.cpp )
target_link_libraries( Adaptive-pyramid ${OpenCV_LIBS} )
