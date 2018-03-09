include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  angle_math
  angle_math.h
  angle_math.cpp)
add_executable(
  angle_math_test
  angle_math_test.cpp)
add_test(angle_math_test angle_math_test)
target_link_libraries (
  angle_math_test
  angle_math
  gtest)

add_library(
  clarke_transformations
  clarke_transformations.h
  clarke_transformations.cpp)
target_link_libraries (
  clarke_transformations
  angle_math)
add_executable(
  clarke_transformations_test
  clarke_transformations_test.cpp)
add_test(clarke_transformations_test clarke_transformations_test)
target_link_libraries (
  clarke_transformations_test
  clarke_transformations
  gtest)

add_library(
  first_order_lag
  first_order_lag.h
  first_order_lag.cpp)
target_link_libraries (
  first_order_lag
  timer)
add_executable(
  first_order_lag_test
  first_order_lag_test.cpp)
add_test(first_order_lag_test first_order_lag_test)
target_link_libraries (
  first_order_lag_test
  first_order_lag
  gtest)

add_library(
  pi
  pi.h
  pi.cpp)
target_link_libraries (
  pi
  timer)
add_executable(
  pi_test
  pi_test.cpp)
add_test(pi_test pi_test)
target_link_libraries (
  pi_test
  pi
  gtest)

add_library(
  integrator
  integrator.h
  integrator.cpp)
target_link_libraries(
  integrator
  timer)
add_executable(
  integrator_test
  integrator_test.cpp)
add_test(integrator_test integrator_test)
target_link_libraries(
  integrator_test
  integrator
  gtest)
