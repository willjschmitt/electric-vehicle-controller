#include "signal_processing/three_phase_dq_filtered.h"

#include <chrono>

#include "control/timer.h"
#include "gtest/gtest.h"
#include "signal_processing/angle_math.h"
#include "signal_processing/clarke_transformations.h"
#include "clarke_transformations.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

using ::electric_vehicle::control::SettableTimer;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::system_clock;

constexpr double kFrequency = 377.0;  // rad/sec

TEST(ThreePhaseDQFiltered, ReturnsInitalValue) {
  SettableTimer timer;
  constexpr double theta = 0.0;
  const double kTau = 10.0;
  ThreePhaseDQFiltered filter(&timer, kTau);
  const DirectQuadrature input_dq{20.0, 40.0};
  const ThreePhase input_three_phase = InverseParkTransformation(input_dq, theta);

  const DirectQuadrature got = filter.Solve(input_three_phase, theta);
  EXPECT_NEAR(got.direct, input_dq.direct, 1E-6);
  EXPECT_NEAR(got.quadrature, input_dq.quadrature, 1E-6);
}

TEST(ThreePhaseDQFiltered, LagsAtTimeConstant) {
  SettableTimer timer;
  double theta = 0.0;
  double current_time = 123.45;
  const double kTau = 1.0;
  ThreePhaseDQFiltered filter(&timer, kTau);
  const DirectQuadrature initial_input_dq{0.0, 0.0};
  ThreePhase input_three_phase = InverseParkTransformation(initial_input_dq, theta);

  timer.SetTime(current_time);
  DirectQuadrature got;
  got = filter.Solve(input_three_phase, theta);
  EXPECT_DOUBLE_EQ(got.direct, initial_input_dq.quadrature);
  EXPECT_DOUBLE_EQ(got.quadrature, initial_input_dq.quadrature);

  const DirectQuadrature input_dq{1.0, 0.5};

  // Make 10 small steps to the first time constant.
  for (unsigned int i = 0; i < 1000; i++) {
    current_time += 0.001;
    theta += 0.001 * kFrequency;
    timer.SetTime(current_time);

    input_three_phase = InverseParkTransformation(input_dq, theta);
    got = filter.Solve(input_three_phase, theta);
  }
  constexpr double kOneTimeConstant = 0.632;
  EXPECT_NEAR(got.direct, kOneTimeConstant, 0.001);
  EXPECT_NEAR(got.quadrature, kOneTimeConstant * 0.5, 0.001);

  // Make 10 small steps to the first time constant.
  for (unsigned int i = 0; i < 1000; i++) {
    current_time += 0.001;
    theta += 0.001 * kFrequency;
    timer.SetTime(current_time);

    input_three_phase = InverseParkTransformation(input_dq, theta);
    got = filter.Solve(input_three_phase, theta);
  }
  constexpr double kTwoTimeConstant = 0.865;
  EXPECT_NEAR(got.direct, kTwoTimeConstant, 0.001);
  EXPECT_NEAR(got.quadrature, kTwoTimeConstant * 0.5, 0.001);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle
