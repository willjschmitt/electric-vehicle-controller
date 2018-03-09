#include "signal_processing/angle_math.h"

#include <cfloat>
#include <cmath>

#include "gtest/gtest.h"
#include "signal_processing/math_constants.h"

namespace electric_vehicle {
namespace signal_processing {
namespace {

constexpr double kSmallError = 1E-9;

TEST(Amplitude, CalculatesAmplitudeOfZero) {
  const TwoDimensionalVector in{ 0.0, 0.0 };
  const double amplitude = Amplitude(in);
  EXPECT_DOUBLE_EQ(amplitude, 0.0);
}

TEST(Amplitude, CalculatesAmplitudeOfXOne) {
  const TwoDimensionalVector in{ 1.0, 0.0 };
  const double amplitude = Amplitude(in);
  EXPECT_DOUBLE_EQ(amplitude, 1.0);
}

TEST(Amplitude, CalculatesAmplitudeOfYOne) {
  const TwoDimensionalVector in{ 0.0, 1.0 };
  const double amplitude = Amplitude(in);
  EXPECT_DOUBLE_EQ(amplitude, 1.0);
}

TEST(Amplitude, CalculatesAmplitudeOfOneByOne) {
  const TwoDimensionalVector in{ 1.0, 1.0 };
  const double amplitude = Amplitude(in);
  EXPECT_DOUBLE_EQ(amplitude, sqrt(2.0));
}

TEST(Rotate, Rotates90To0) {
  const TwoDimensionalVector unrotated{ 0.0, 1.0 };
  const TwoDimensionalVector rotated = Rotate(unrotated, -kPiBy2);
  EXPECT_NEAR(rotated.x, 1.0, kSmallError);
  EXPECT_NEAR(rotated.y, 0.0, kSmallError);
}

TEST(Rotate, Rotates0To90) {
  const TwoDimensionalVector unrotated{ 1.0, 0.0 };
  const TwoDimensionalVector rotated = Rotate(unrotated, +kPiBy2);
  EXPECT_NEAR(rotated.x, 0.0, kSmallError);
  EXPECT_NEAR(rotated.y, 1.0, kSmallError);
}

TEST(Rotate, Rotates45To115) {
  const TwoDimensionalVector unrotated{ 0.5, 0.5 };
  const TwoDimensionalVector rotated = Rotate(unrotated, +kPiBy2);
  EXPECT_NEAR(rotated.x, -0.5, kSmallError);
  EXPECT_NEAR(rotated.y, 0.5, kSmallError);
}

TEST(Scale, ScalesAVector) {
  const TwoDimensionalVector unscaled{ 0.5, 1.0 };
  const double kGain = 2.0;
  const TwoDimensionalVector scaled = Scale(unscaled, kGain);
  EXPECT_DOUBLE_EQ(scaled.x, 1.0);
  EXPECT_DOUBLE_EQ(scaled.y, 2.0);
}

TEST(Limit, LimitsVectorPreservingAngle) {
  // Make a 3-4-5 triangle doubled, with it limited to 5, so we get a 3-4-5
  // out.
  const TwoDimensionalVector unlimited{ 6.0, 8.0 };
  const double kLimit = 5.0;
  const TwoDimensionalVector limited = Limit(unlimited, kLimit);
  EXPECT_DOUBLE_EQ(limited.x, 3.0);
  EXPECT_DOUBLE_EQ(limited.y, 4.0);
}

TEST(Limit, LeavesVectorAlone) {
  const TwoDimensionalVector unlimited{0.5, 1.0};
  const double kLimit = DBL_MAX;
  const TwoDimensionalVector limited = Limit(unlimited, kLimit);
  EXPECT_DOUBLE_EQ(limited.x, 0.5);
  EXPECT_DOUBLE_EQ(limited.y, 1.0);
}

}  // namespace
}  // namespace signal_processing
}  // namespace electric_vehicle

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
