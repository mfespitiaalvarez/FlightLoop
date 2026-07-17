// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/frames/rotation.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>
#include <random>
#include <vector>

namespace flightloop {
namespace {

constexpr double kPi = std::numbers::pi;

// Fixed seed so failures reproduce; these are property tests over a
// deterministic sample, not fuzz tests.
constexpr unsigned int kSeed = 20260716;
constexpr int kNumSamples = 500;

// Smallest signed difference between two angles, so values that differ by a
// multiple of 2*pi (e.g. pi vs -pi) compare as equal.
double AngleDiff(double a, double b) { return std::remainder(a - b, 2 * kPi); }

// Samples Euler angles over their full documented ranges except pitch, which
// stays `pitch_margin` radians clear of +/-pi/2: yaw/roll extraction is
// ill-conditioned there (error grows like eps / cos(pitch)), and the
// gimbal-lock convention is covered by dedicated tests instead.
std::vector<Euler321> SampleEulerAnglesAwayFromLock(double pitch_margin) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  std::uniform_real_distribution<double> pitch(-kPi / 2 + pitch_margin,
                                               kPi / 2 - pitch_margin);
  std::vector<Euler321> samples;
  samples.reserve(kNumSamples);
  for (int i = 0; i < kNumSamples; ++i) {
    samples.push_back({angle(rng), pitch(rng), angle(rng)});
  }
  return samples;
}

// Spot-checks the passive convention v_N = q_NB * v_B on attitudes where the
// answer is known by inspection (NED: x north, y east, z down).
TEST(RotationTest, KnownAttitudesMapBodyAxesToExpectedNedDirections) {
  const Eigen::Vector3d body_x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d body_y = Eigen::Vector3d::UnitY();

  // Zero attitude: body axes coincide with NED.
  const Eigen::Quaterniond q_identity =
      QuaternionFromEuler321({.yaw = 0.0, .pitch = 0.0, .roll = 0.0});
  EXPECT_LT((q_identity * body_x - Eigen::Vector3d::UnitX()).norm(), 1e-14);

  // Yaw +90 deg: the nose (body x) points east.
  const Eigen::Quaterniond q_yaw =
      QuaternionFromEuler321({.yaw = kPi / 2, .pitch = 0.0, .roll = 0.0});
  EXPECT_LT((q_yaw * body_x - Eigen::Vector3d::UnitY()).norm(), 1e-14);

  // Pitch +90 deg: the nose points up (-z in NED).
  const Eigen::Quaterniond q_pitch =
      QuaternionFromEuler321({.yaw = 0.0, .pitch = kPi / 2, .roll = 0.0});
  EXPECT_LT((q_pitch * body_x + Eigen::Vector3d::UnitZ()).norm(), 1e-14);

  // Roll +90 deg: the right wing (body y) points down (+z in NED).
  const Eigen::Quaterniond q_roll =
      QuaternionFromEuler321({.yaw = 0.0, .pitch = 0.0, .roll = kPi / 2});
  EXPECT_LT((q_roll * body_y - Eigen::Vector3d::UnitZ()).norm(), 1e-14);
}

TEST(RotationTest, QuaternionFromEuler321ReturnsUnitQuaternion) {
  for (const Euler321& e : SampleEulerAnglesAwayFromLock(0.0)) {
    const Eigen::Quaterniond q = QuaternionFromEuler321(e);
    EXPECT_NEAR(q.norm(), 1.0, 1e-14)
        << "yaw=" << e.yaw << " pitch=" << e.pitch << " roll=" << e.roll;
  }
}

// With pitch 0.05 rad clear of the singularity the round-trip conditioning
// factor 1/cos(pitch) is at most ~20, so double round-off (~1e-16) leaves
// ample margin under a 1e-12 tolerance.
TEST(RotationTest, RoundTripRecoversAnglesAwayFromGimbalLock) {
  for (const Euler321& e : SampleEulerAnglesAwayFromLock(0.05)) {
    const Euler321 out = Euler321FromQuaternion(QuaternionFromEuler321(e));
    EXPECT_LT(std::abs(AngleDiff(out.yaw, e.yaw)), 1e-12);
    EXPECT_LT(std::abs(AngleDiff(out.pitch, e.pitch)), 1e-12);
    EXPECT_LT(std::abs(AngleDiff(out.roll, e.roll)), 1e-12)
        << "yaw=" << e.yaw << " pitch=" << e.pitch << " roll=" << e.roll;
  }
}

// Eigen 5's canonicalEulerAngles(2, 1, 0) computes the same intrinsic 3-2-1
// decomposition with the same Tait-Bryan ranges, so away from gimbal lock the
// hand-rolled extraction must agree with it angle for angle. (At the poles
// Eigen returns an arbitrary valid alias, so the roll = 0 convention is
// checked separately.)
TEST(RotationTest, MatchesEigenCanonicalEulerAnglesAwayFromGimbalLock) {
  for (const Euler321& e : SampleEulerAnglesAwayFromLock(0.05)) {
    const Eigen::Matrix3d R = QuaternionFromEuler321(e).toRotationMatrix();
    const Euler321 out = Euler321FromDcm(R);
    const Eigen::Vector3d oracle = R.canonicalEulerAngles(2, 1, 0);
    EXPECT_LT(std::abs(AngleDiff(out.yaw, oracle[0])), 1e-12);
    EXPECT_LT(std::abs(AngleDiff(out.pitch, oracle[1])), 1e-12);
    EXPECT_LT(std::abs(AngleDiff(out.roll, oracle[2])), 1e-12)
        << "yaw=" << e.yaw << " pitch=" << e.pitch << " roll=" << e.roll;
  }
}

// At pitch = +pi/2 only yaw - roll is observable; the convention pins
// roll = 0 with the difference absorbed into yaw. The extracted alias must
// still represent the same rotation.
TEST(RotationTest, GimbalLockNoseUpSetsRollZeroAndAbsorbsIntoYaw) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  for (int i = 0; i < kNumSamples; ++i) {
    const Euler321 e{angle(rng), kPi / 2, angle(rng)};
    const Eigen::Quaterniond q = QuaternionFromEuler321(e);
    const Euler321 out = Euler321FromQuaternion(q);
    EXPECT_DOUBLE_EQ(out.roll, 0.0);
    EXPECT_DOUBLE_EQ(out.pitch, kPi / 2);
    EXPECT_LT(std::abs(AngleDiff(out.yaw, e.yaw - e.roll)), 1e-12)
        << "yaw=" << e.yaw << " roll=" << e.roll;
    EXPECT_LT(QuaternionFromEuler321(out).angularDistance(q), 1e-12);
  }
}

// At pitch = -pi/2 the observable combination flips to yaw + roll.
TEST(RotationTest, GimbalLockNoseDownSetsRollZeroAndAbsorbsIntoYaw) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  for (int i = 0; i < kNumSamples; ++i) {
    const Euler321 e{angle(rng), -kPi / 2, angle(rng)};
    const Eigen::Quaterniond q = QuaternionFromEuler321(e);
    const Euler321 out = Euler321FromQuaternion(q);
    EXPECT_DOUBLE_EQ(out.roll, 0.0);
    EXPECT_DOUBLE_EQ(out.pitch, -kPi / 2);
    EXPECT_LT(std::abs(AngleDiff(out.yaw, e.yaw + e.roll)), 1e-12)
        << "yaw=" << e.yaw << " roll=" << e.roll;
    EXPECT_LT(QuaternionFromEuler321(out).angularDistance(q), 1e-12);
  }
}

// A DCM with exact zeros at the singularity (pure pitch up, Ry(pi/2)) must
// take the gimbal-lock branch and return the pinned convention exactly.
TEST(RotationTest, GimbalLockExactDcmReturnsPinnedConvention) {
  Eigen::Matrix3d R;
  R << 0, 0, 1,  //
      0, 1, 0,   //
      -1, 0, 0;
  const Euler321 out = Euler321FromDcm(R);
  EXPECT_DOUBLE_EQ(out.yaw, 0.0);
  EXPECT_DOUBLE_EQ(out.pitch, kPi / 2);
  EXPECT_DOUBLE_EQ(out.roll, 0.0);
}

// Just inside the gimbal-lock threshold the individual yaw/roll values are
// ill-conditioned and may not match the inputs, but whatever alias comes out
// must still reconstruct the original rotation. Extraction error is roughly
// eps / cos(pitch) ~ 1e-9 rad here; 1e-7 leaves margin.
TEST(RotationTest, NearGimbalLockStillReconstructsTheRotation) {
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  for (const double pitch : {kPi / 2 - 1e-7, -kPi / 2 + 1e-7}) {
    for (int i = 0; i < kNumSamples; ++i) {
      const Euler321 e{angle(rng), pitch, angle(rng)};
      const Eigen::Quaterniond q = QuaternionFromEuler321(e);
      const Euler321 out = Euler321FromQuaternion(q);
      EXPECT_LT(QuaternionFromEuler321(out).angularDistance(q), 1e-7)
          << "yaw=" << e.yaw << " pitch=" << e.pitch << " roll=" << e.roll;
    }
  }
}

// q and -q are the same rotation; extraction must not depend on the sign.
// toRotationMatrix() is quadratic in the components, so the results should be
// bitwise identical, not merely close.
TEST(RotationTest, NegatedQuaternionYieldsIdenticalAngles) {
  for (const Euler321& e : SampleEulerAnglesAwayFromLock(0.0)) {
    const Eigen::Quaterniond q = QuaternionFromEuler321(e);
    const Eigen::Quaterniond q_negated(-q.w(), -q.x(), -q.y(), -q.z());
    const Euler321 a = Euler321FromQuaternion(q);
    const Euler321 b = Euler321FromQuaternion(q_negated);
    EXPECT_EQ(a.yaw, b.yaw);
    EXPECT_EQ(a.pitch, b.pitch);
    EXPECT_EQ(a.roll, b.roll);
  }
}

// Uniformly random attitudes (normalized Gaussian quaternions) must extract
// into the documented ranges. atan2 can return exactly -pi for signed-zero
// edge cases, so yaw/roll are checked against the closed interval [-pi, pi]
// rather than the header's (-pi, pi].
TEST(RotationTest, ExtractedAnglesLieInDocumentedRanges) {
  std::mt19937 rng(kSeed);
  std::normal_distribution<double> normal;
  for (int i = 0; i < kNumSamples; ++i) {
    Eigen::Quaterniond q(normal(rng), normal(rng), normal(rng), normal(rng));
    q.normalize();
    const Euler321 out = Euler321FromQuaternion(q);
    EXPECT_GE(out.yaw, -kPi);
    EXPECT_LE(out.yaw, kPi);
    EXPECT_GE(out.pitch, -kPi / 2);
    EXPECT_LE(out.pitch, kPi / 2);
    EXPECT_GE(out.roll, -kPi);
    EXPECT_LE(out.roll, kPi);
  }
}

}  // namespace
}  // namespace flightloop
