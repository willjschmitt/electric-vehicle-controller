include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  current_regulator
  current_regulator.h
  current_regulator.cc)
target_link_libraries(
  current_regulator
  pi)
add_executable(
  current_regulator_test
  current_regulator_test.cc)
add_test(current_regulator_test current_regulator_test)
target_link_libraries (
  current_regulator_test
  current_regulator
  gtest
  timer)

add_library(
  timer
  timer.h
  timer.cc)
add_executable(
  timer_test
  timer_test.cc)
add_test(timer_test timer_test)
target_link_libraries(
  timer_test
  timer
  gtest)

add_library(
  modulation
  modulation.h
  modulation.cc)
target_link_libraries(
  modulation
  clarke_transformations)
add_executable(
  modulation_test
  modulation_test.cc)
add_test(modulation_test modulation_test)
target_link_libraries(
  modulation_test
  modulation
  clarke_transformations
  gtest)