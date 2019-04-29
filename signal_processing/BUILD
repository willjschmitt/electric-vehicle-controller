package(default_visibility = ["//visibility:public"])

cc_library(
  name = "math_constants",
  hdrs = ["math_constants.h"],
)

cc_library(
  name = "angle_math",
  srcs = ["angle_math.cpp"],
  hdrs = ["angle_math.h"],
)

cc_test(
  name = "angle_math_test",
  srcs = ["angle_math_test.cpp"],
  deps = [
    ":angle_math",
    ":math_constants",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "clarke_transformations",
  srcs = ["clarke_transformations.cpp"],
  hdrs = ["clarke_transformations.h"],
  deps = [
    ":angle_math",
    ":math_constants",
  ]
)

cc_test(
  name = "clarke_transformations_test",
  srcs = ["clarke_transformations_test.cpp"],
  deps = [
    ":clarke_transformations",
    ":math_constants",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "first_order_lag",
  srcs = ["first_order_lag.cpp"],
  hdrs = ["first_order_lag.h"],
  deps = [
    "//control:timer",
  ]
)

cc_test(
  name = "first_order_lag_test",
  srcs = ["first_order_lag_test.cpp"],
  deps = [
    ":first_order_lag",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "integrator",
  srcs = ["integrator.cpp"],
  hdrs = ["integrator.h"],
  deps = [
    "//control:timer",
  ]
)

cc_test(
  name = "integrator_test",
  srcs = ["integrator_test.cpp"],
  deps = [
    ":integrator",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)

cc_library(
  name = "pi",
  srcs = ["pi.cpp"],
  hdrs = ["pi.h"],
  deps = [
    "//control:timer",
  ]
)

cc_test(
  name = "pi_test",
  srcs = ["pi_test.cpp"],
  deps = [
    ":pi",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)
