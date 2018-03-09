#include "signal_processing/clarke_transformations.h"

#include <cmath>

#include "gtest/gtest.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

constexpr double kSmallError = 1E-9;

TEST(ThreePhase, SubscriptOperatorWithConstReturns) {
  const double kA = 0.1;
  const double kB = 0.2;
  const double kC = 0.3;
  const ThreePhase three_phase{kA, kB, kC};
  const double& got_a = three_phase[0];
  const double& got_b = three_phase[1];
  const double& got_c = three_phase[2];
  EXPECT_DOUBLE_EQ(got_a, kA);
  EXPECT_DOUBLE_EQ(got_b, kB);
  EXPECT_DOUBLE_EQ(got_c, kC);
}

TEST(ThreePhase, SubscriptOperatorWithNonConstReturns) {
  const double kA = 0.1;
  const double kB = 0.2;
  const double kC = 0.3;
  ThreePhase three_phase{kA, kB, kC};
  double& got_a = three_phase[0];
  double& got_b = three_phase[1];
  double& got_c = three_phase[2];
  EXPECT_DOUBLE_EQ(got_a, kA);
  EXPECT_DOUBLE_EQ(got_b, kB);
  EXPECT_DOUBLE_EQ(got_c, kC);
}

TEST(ThreePhase, SubscriptOperatorIsSettable) {
  const double kA = 0.1;
  const double kB = 0.2;
  const double kC = 0.3;
  ThreePhase three_phase{kA, kB, kC};
  // Shift the order of the values by one to check if it persists.
  three_phase[0] = kB;
  three_phase[1] = kC;
  three_phase[2] = kA;
  const double& got_a = three_phase[0];
  const double& got_b = three_phase[1];
  const double& got_c = three_phase[2];
  EXPECT_DOUBLE_EQ(got_a, kB);
  EXPECT_DOUBLE_EQ(got_b, kC);
  EXPECT_DOUBLE_EQ(got_c, kA);
}

TEST(DirectQuadrature, SubtractOperator) {
  const DirectQuadrature a{ 1.0, 2.0, 3.0 };
  const DirectQuadrature b{ -1.0, -2.0, -3.0 };
  const DirectQuadrature c = a - b;
  EXPECT_DOUBLE_EQ(c.direct, 2.0);
  EXPECT_DOUBLE_EQ(c.quadrature, 4.0);
  EXPECT_DOUBLE_EQ(c.zero, 6.0);
}

TEST(DirectQuadrature, AdditionOperator) {
  const DirectQuadrature a{ 1.0, 2.0, 3.0 };
  const DirectQuadrature b{ 2.0, 3.0, 4.0 };
  const DirectQuadrature c = a + b;
  EXPECT_DOUBLE_EQ(c.direct, 3.0);
  EXPECT_DOUBLE_EQ(c.quadrature, 5.0);
  EXPECT_DOUBLE_EQ(c.zero, 7.0);
}

TEST(ClarkeTransformation, CalculatesAngle0) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, kMagnitude, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, 0.0, kSmallError);
}

TEST(ClarkeTransformation, CalculatesAngle90) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, kMagnitude, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, 0.0, kSmallError);
}

TEST(ClarkeTransformation, CalculatesZeroSequence) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.A() = kMagnitude;
  abc.B() = kMagnitude;
  abc.C() = kMagnitude;
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  EXPECT_NEAR(alpha_beta.alpha, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.beta, 0.0, kSmallError);
  EXPECT_NEAR(alpha_beta.gamma, kMagnitude, kSmallError);
}

TEST(ParkTransformation, CalculateAngle0) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 0.0;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateAngle90) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateAngle270) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = 3.0 * kPiBy2;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, kMagnitude, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, 0.0, kSmallError);
}

TEST(ParkTransformation, CalculateZeroSequence) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.A() = kMagnitude;
  abc.B() = kMagnitude;
  abc.C() = kMagnitude;
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  EXPECT_NEAR(dq0.direct, 0.0, kSmallError);
  EXPECT_NEAR(dq0.quadrature, 0.0, kSmallError);
  EXPECT_NEAR(dq0.zero, kMagnitude, kSmallError);
}

TEST(InverseClarkeTransformation, ClarkeTransformationIsReversible) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const AlphaBeta alpha_beta = ClarkeTransformation(abc);
  const ThreePhase got = InverseClarkeTransformation(alpha_beta);
  EXPECT_NEAR(got.A(), abc.A(), kSmallError);
  EXPECT_NEAR(got.B(), abc.B(), kSmallError);
  EXPECT_NEAR(got.C(), abc.C(), kSmallError);
}

TEST(InverseParkTransformation, ParkTransformationIsReversible) {
  ThreePhase abc;
  const double kMagnitude = 1.0;
  const double kTheta = kPiBy2;
  abc.A() = kMagnitude * cos(kTheta);
  abc.B() = kMagnitude * cos(kTheta - k2PiBy3);
  abc.C() = kMagnitude * cos(kTheta + k2PiBy3);
  const DirectQuadrature dq0 = ParkTransformation(abc, kTheta);
  const ThreePhase got = InverseParkTransformation(dq0, kTheta);
  EXPECT_NEAR(got.A(), abc.A(), kSmallError);
  EXPECT_NEAR(got.B(), abc.B(), kSmallError);
  EXPECT_NEAR(got.C(), abc.C(), kSmallError);
}


}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
