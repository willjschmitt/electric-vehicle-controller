package(default_visibility = ["//visibility:public"])

cc_library(
  name = "dc_voltage",
  srcs = ["dc_voltage.cpp"],
  hdrs = ["dc_voltage.h"],
)

cc_test(
  name = "dc_voltage_test",
  srcs = ["dc_voltage_test.cpp"],
  deps = [
    ":dc_voltage",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "mechanical_speed",
  srcs = ["mechanical_speed.cpp"],
  hdrs = ["mechanical_speed.h"],
)

cc_test(
  name = "mechanical_speed_test",
  srcs = ["mechanical_speed_test.cpp"],
  deps = [
    ":mechanical_speed",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "three_phase_currents",
  srcs = ["three_phase_currents.cpp"],
  hdrs = ["three_phase_currents.h"],
  deps = [
    "//signal_processing:clarke_transformations",
  ]
)

cc_test(
  name = "three_phase_currents_test",
  srcs = ["three_phase_currents_test.cpp"],
  deps = [
    ":three_phase_currents",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "throttle",
  srcs = ["throttle.cpp"],
  hdrs = ["throttle.h"],
)

cc_test(
  name = "throttle_test",
  srcs = ["throttle_test.cpp"],
  deps = [
    ":throttle",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)
