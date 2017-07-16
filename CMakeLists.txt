cmake_minimum_required(VERSION 3.1)

if (COMPILE_PIC)
  set(CMAKE_TOOLCHAIN_FILE cmake-microchip/toolchain.cmake)
  set(MICROCHIP_MCU PIC30F6015)
endif ()

project( electric-vehicle-controller )

include(CTest)
enable_testing()

set (CMAKE_CXX_STANDARD 11)

add_subdirectory(googletest)

add_subdirectory(control)
add_subdirectory(machines)
add_subdirectory(measurement)
add_subdirectory(signal_processing)
