// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/integrator/rk4_integrator.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "flightloop/ode/rhs.h"

namespace flightloop {
namespace {

TEST(Rk4IntegratorTest, ConstructorRejectsNonPositiveSize) {
  EXPECT_THROW(Rk4Integrator(0), std::invalid_argument);
  EXPECT_THROW(Rk4Integrator(-1), std::invalid_argument);
}

// x_dot = -x with x(0) = 1 has the exact solution x(t) = e^-t. RK4's global
// error is O(dt^4); for this problem it is roughly T * dt^4 / 120, so with
// dt = 0.01 the answer should be within ~1e-10 of exact. The 1e-9 tolerance
// leaves margin while staying far below what a lower-order method could hit
// (forward Euler lands ~5e-3 away at this dt).
TEST(Rk4IntegratorTest, MatchesExactExponentialDecaySolution) {
  constexpr double kFinalTime = 1.0;
  constexpr int kNumSteps = 100;
  constexpr double kDt = kFinalTime / kNumSteps;

  Rk4Integrator integrator(1);
  const RhsFunction rhs =
      [](double /*t*/, const Eigen::VectorXd& x, Eigen::VectorXd& x_dot) {
        x_dot = -x;
        return RhsStatus::kOk;
      };

  Eigen::VectorXd x(1);
  x << 1.0;
  double t = 0.0;
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(integrator.Step(t, kDt, rhs, x), Integrator::StepStatus::kOk);
    t += kDt;
  }

  EXPECT_NEAR(x(0), std::exp(-kFinalTime), 1e-9);
}

// TODO(mfespitiaalvarez): 2-D simple harmonic oscillator as a first-order
// system:
//   state = [position, velocity]
//   x_dot(0) = state(1)
//   x_dot(1) = -omega^2 * state(0)
// From initial state [1, 0] the exact solution is
//   position(t) = cos(omega * t),  velocity(t) = -omega * sin(omega * t).
// This exercises n > 1 (buffers sized for arbitrary n) and coupling between
// components, which the scalar decay test cannot see.
TEST(Rk4IntegratorTest, MatchesExactHarmonicOscillatorSolution) {
  GTEST_SKIP() << "Not implemented yet";
  // 1. Construct Rk4Integrator(2) and an RhsFunction for the system above.
  // 2. Integrate from t = 0 to a final time over fixed steps, ASSERT-ing
  //    kOk on every step.
  // 3. EXPECT_NEAR each state component against the exact solution, with a
  //    tolerance justified the same way as the decay test's.
}

}  // namespace
}  // namespace flightloop
