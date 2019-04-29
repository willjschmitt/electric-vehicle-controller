package(default_visibility = ["//visibility:public"])

cc_library(
  name = "current_regulator",
  srcs = ["current_regulator.cpp"],
  hdrs = ["current_regulator.h"],
  deps = [
    ":timer",
    "//signal_processing:clarke_transformations",
    "//signal_processing:pi",
  ]
)

cc_test(
  name = "current_regulator_test",
  srcs = ["current_regulator_test.cpp"],
  deps = [
    ":current_regulator",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "modulation",
  srcs = [
    "modulation.cpp",
    "modulation_commands.cpp",
  ],
  hdrs = [
    "modulation.h",
    "modulation_commands.h",
  ],
  deps = [
    ":timer",
    "//signal_processing:clarke_transformations",
  ]
)

cc_test(
  name = "modulation_test",
  srcs = [
    "modulation_commands_test.cpp",
    "modulation_test.cpp",
  ],
  deps = [
    ":modulation",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "stator_angle_estimator",
  srcs = ["stator_angle_estimator.cpp"],
  hdrs = ["stator_angle_estimator.h"],
  deps = [
    ":timer",
    "//signal_processing:math_constants",
    "//signal_processing:integrator",
  ]
)

cc_test(
  name = "stator_angle_estimator_test",
  srcs = ["stator_angle_estimator_test.cpp"],
  deps = [
    ":stator_angle_estimator",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)


cc_library(
  name = "timer",
  srcs = ["timer.cpp"],
  hdrs = ["timer.h"],
)

cc_test(
  name = "timer_test",
  srcs = ["timer_test.cpp"],
  deps = [
    ":timer",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)
