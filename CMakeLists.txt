cmake_minimum_required(VERSION 3.1)

project( electric-vehicle-controller )

include(CTest)
enable_testing()

set (CMAKE_CXX_STANDARD 11)

add_subdirectory(googletest)
add_subdirectory(signal_processing)
