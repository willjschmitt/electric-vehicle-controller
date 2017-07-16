include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  three_phase_currents
  three_phase_currents.h
  three_phase_currents.cc)
add_executable(
  three_phase_currents_test
  three_phase_currents_test.cc)
add_test(three_phase_currents_test three_phase_currents_test)
target_link_libraries(
  three_phase_currents_test
  three_phase_currents
  gtest)

add_library(
  dc_voltage
  dc_voltage.h
  dc_voltage.cc)
add_executable(
  dc_voltage_test
  dc_voltage_test.cc)
add_test(dc_voltage_test dc_voltage_test)
target_link_libraries(
  dc_voltage_test
  dc_voltage
  gtest)

add_library(
  throttle
  throttle.h
  throttle.cc)
add_executable(
  throttle_test
  throttle_test.cc)
add_test(throttle_test throttle_test)
target_link_libraries(
  throttle_test
  throttle
  gtest)

add_library(
  mechanical_speed
  mechanical_speed.h
  mechanical_speed.cc)
add_executable(
  mechanical_speed_test
  mechanical_speed_test.cc)
add_test(mechanical_speed_test mechanical_speed_test)
target_link_libraries(
  mechanical_speed_test
  mechanical_speed
  gtest)
