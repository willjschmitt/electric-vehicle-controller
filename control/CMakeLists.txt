include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  current_regulator
  current_regulator.h
  current_regulator.cpp)
target_link_libraries(
  current_regulator
  pi)
add_executable(
  current_regulator_test
  current_regulator_test.cpp)
add_test(current_regulator_test current_regulator_test)
target_link_libraries (
  current_regulator_test
  current_regulator
  gtest
  timer)

add_library(
  timer
  timer.h
  timer.cpp)
add_executable(
  timer_test
  timer_test.cpp)
add_test(timer_test timer_test)
target_link_libraries(
  timer_test
  timer
  gtest)

add_library(
  modulation
  modulation.h
  modulation.cpp)
target_link_libraries(
  modulation
  clarke_transformations)
add_executable(
  modulation_test
  modulation_test.cpp)
add_test(modulation_test modulation_test)
target_link_libraries(
  modulation_test
  modulation
  clarke_transformations
  gtest)

add_library(
  stator_angle_estimator
  stator_angle_estimator.h
  stator_angle_estimator.cpp)
target_link_libraries(
  stator_angle_estimator
  clarke_transformations
  integrator
  timer)
add_executable(
  stator_angle_estimator_test
  stator_angle_estimator_test.cpp)
add_test(stator_angle_estimator_test stator_angle_estimator_test)
target_link_libraries(
  stator_angle_estimator_test
  stator_angle_estimator
  gtest)
