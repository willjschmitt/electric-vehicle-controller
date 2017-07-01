#include "signal_processing/clarke_transformations.h"

#include <cmath>

#include "gtest/gtest.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace signal_processing {

constexpr double kSmallError = 1E-9;

TEST(ClarkeTransformation, CalculatesAngle0) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.a = kMagnitude * cos(kTheta);
  abc.b = kMagnitude * cos(kTheta - k2PiBy3);
  abc.c = kMagnitude * cos(kTheta + k2PiBy3);
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, kMagnitude, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, 0.0, kSmallError);
}

TEST(ClarkeTransformation, CalculatesAngle90) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.a = kMagnitude * cos(kTheta);
  abc.b = kMagnitude * cos(kTheta - k2PiBy3);
  abc.c = kMagnitude * cos(kTheta + k2PiBy3);
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, kMagnitude, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, 0.0, kSmallError);
}

TEST(ClarkeTransformation, CalculatesZeroSequence) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.a = kMagnitude;
  abc.b = kMagnitude;
  abc.c = kMagnitude;
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, kMagnitude, kSmallError);
}

TEST(ParkTransformation, CalculateAngle0) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.a = kMagnitude * cos(kTheta);
  abc.b = kMagnitude * cos(kTheta - k2PiBy3);
  abc.c = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateAngle90) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.a = kMagnitude * cos(kTheta);
  abc.b = kMagnitude * cos(kTheta - k2PiBy3);
  abc.c = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateAngle270) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 3.0 * kPiBy2;
  abc.a = kMagnitude * cos(kTheta);
  abc.b = kMagnitude * cos(kTheta - k2PiBy3);
  abc.c = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateZeroSequence) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.a = kMagnitude;
  abc.b = kMagnitude;
  abc.c = kMagnitude;
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, 0.0, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, kMagnitude, kSmallError);
}

}  // namespace signal_processing
}  // namespace electric_vehicle
