cmake_minimum_required(VERSION 2.6.4)

project( electric-vehicle-controller )

include(CTest)
enable_testing()

add_subdirectory(googletest)
add_subdirectory(signal_processing)
