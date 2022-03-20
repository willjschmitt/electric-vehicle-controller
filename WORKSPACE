load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.8.1.zip",
    strip_prefix = "googletest-release-1.8.1",
    sha256 = "927827c183d01734cc5cfef85e0ff3f5a92ffe6188e0d18e909c5efebf28a0c7"
)

# This is a dependency of gtest for many of its targets. We don't strictly need
# this, but it is important for being able to query strongly the dependencies on
# gtest to exclude them for xc32 builds, since gtest is not compilable with
# xc32.
http_archive(
  name = "com_google_absl",
  sha256 = "aabf6c57e3834f8dc3873a927f37eaf69975d4b28117fc7427dfb1c661542a87",
  urls = ["https://github.com/abseil/abseil-cpp/archive/98eb410c93ad059f9bba1bf43f5bb916fc92a5ea.zip"],
  strip_prefix = "abseil-cpp-98eb410c93ad059f9bba1bf43f5bb916fc92a5ea",
)

_RULES_MICROCHIP_COMMIT_SHA = "3c3829d63e929a58e5daa1da2d8433b6307f19eb"

# git_repository is not compatible here, since the x-bit is dropped on our
# wrapper scripts.
http_archive(
    name = "rules_microchip",
    sha256 = "e321777fb191b8dd888953587e3a6b3a9dfc96c4b4dae98cea50789171603a81",
    url = "https://github.com/willjschmitt/rules_microchip/archive/{}.zip".format(_RULES_MICROCHIP_COMMIT_SHA),
    strip_prefix = "rules_microchip-{}".format(_RULES_MICROCHIP_COMMIT_SHA),
)

load("@rules_microchip//:deps.bzl", "microchip_repositories", "dfp_repositories")

microchip_repositories()
dfp_repositories()
