# electric-vehicle-controller
[![willjschmitt](https://circleci.com/gh/willjschmitt/electric-vehicle-controller.svg?style=shield)](<https://circleci.com/gh/willjschmitt/electric-vehicle-controller>)

AC Induction Motor Controller for an Electric Vehicle

Work In Progress

DQ Control for AC induction motors in electric vehicle applications.
Configurable control ratings based on ratings and limits of components and
electrical properties of motor.

## Getting started
This project is still a work in progress, so instructions are limited to
development efforts for the time being.

The project uses [Bazel](https://bazel.build/) for building and testing, with
[GoogleTest](https://github.com/google/googletest) for a testing framework.

With Bazel installed on a build environment supporting C++ compilation, tests
can be run:
```
bazel test //...
```
