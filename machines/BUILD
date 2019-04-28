package(default_visibility = ["//visibility:public"])

cc_library(
  name = "induction_machine",
  srcs = ["induction_machine.cpp"],
  hdrs = ["induction_machine.h"],
)

cc_test(
  name = "induction_machine_test",
  srcs = ["induction_machine_test.cpp"],
  deps = [
    ":induction_machine",
    "@gtest//:gtest",
    "@gtest//:gtest_main"
  ],
)
