# electric-vehicle-controller
[![Build Status](https://travis-ci.org/willjschmitt/electric-vehicle-controller.svg?branch=master)](https://travis-ci.org/willjschmitt/electric-vehicle-controller)

AC Induction Motor Controller for an Electric Vehicle

Work In Progress

DQ Control for AC induction motors in electric vehicle applications. Configurable control ratings based on ratings and limits of components and electrical properties of motor.

## Getting started
This project is still a work in progress, so instructions are limited to development efforts for the time being.

The project uses [CMake](https://cmake.org/) for building and testing, with [GoogleTest](https://github.com/google/googletest) for a testing framework (Git Submodule). With CMake installed and configured for your favorite build environment:
```
cmake .
make # or your build system's equivalent.
ctest # --verbose (for detailed test failures).
```
