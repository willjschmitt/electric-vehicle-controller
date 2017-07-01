include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

add_library(
  signal_processing
  math_constants.h
  pi.h pi.cc)

add_executable(
  signal_processing_test
  signal_processing_test.cc
  pi_test.cc)

include(CTest)
enable_testing()

add_test(signal_processing_test signal_processing_test)

target_link_libraries (signal_processing_test
  gtest
  signal_processing)
