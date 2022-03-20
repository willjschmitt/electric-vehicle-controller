cc_library(
  name = "induction_motor_controller",
  srcs = ["induction_motor_controller.cpp"],
  visibility = [
      "//:__pkg__",
  ],
  hdrs = ["induction_motor_controller.h"],
  deps = [
    "//control:current_regulator",
    "//control:modulation",
    "//control:stator_angle_estimator",
    "//machines:induction_machine",
    "//measurement:dc_voltage",
    "//measurement:mechanical_speed",
    "//measurement:three_phase_currents",
    "//measurement:throttle",
  ]
)

cc_test(
  name = "induction_motor_controller_test",
  srcs = ["induction_motor_controller_test.cpp"],
  deps = [
    ":induction_motor_controller",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)
