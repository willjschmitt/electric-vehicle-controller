#include "signal_processing/clarke_transformations.h"

#include <cmath>

#include "gtest/gtest.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

constexpr double kSmallError = 1E-9;

TEST(ThreePhase, ToVector) {
  const ThreePhase three_phase_struct{0.1, 0.2, 0.3};
  const std::vector<double> three_phase_vector = three_phase_struct.ToVector();
  EXPECT_EQ(three_phase_vector[0], three_phase_struct.a);
  EXPECT_EQ(three_phase_vector[1], three_phase_struct.b);
  EXPECT_EQ(three_phase_vector[2], three_phase_struct.c);
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

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
