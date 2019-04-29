include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/googletest/googletest/include)

include(${CMAKE_SOURCE_DIR}/googletest/googletest/cmake/internal_utils.cmake)
config_compiler_and_linker()

include(CTest)
enable_testing()

add_library(
  induction_motor_controller
  induction_motor_controller.h
  induction_motor_controller.cpp)
target_link_libraries(
  induction_motor_controller
  current_regulator
  induction_machine
  integrator
  mechanical_speed
  modulation
  stator_angle_estimator
  three_phase_currents
  throttle
  timer)
add_executable(
  induction_motor_controller_test
  induction_motor_controller_test.cpp)
add_test(induction_motor_controller_test induction_motor_controller_test)
target_link_libraries(
  induction_motor_controller_test
  induction_motor_controller
  gtest
  gmock)
