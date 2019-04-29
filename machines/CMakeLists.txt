include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  induction_machine
  induction_machine.h
  induction_machine.cpp)
add_executable(
  induction_machine_test
  induction_machine_test.cpp)
add_test(induction_machine_test induction_machine_test)
target_link_libraries (
  induction_machine_test
  induction_machine
  gtest
  gmock)
