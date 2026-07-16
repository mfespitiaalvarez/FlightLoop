// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// Numerical time integrators that advance a vehicle's continuous state one
// step.
//
// Declares the Integrator abstraction: the time-stepping contract that the
// concrete strategies (RK4, RK45, and the optional CVODE wrapper) satisfy. The
// right-hand side is supplied per call as a pure function of (t, x, u), so
// integrators stay agnostic to any vehicle or force models.

#ifndef FLIGHTLOOP_INTEGRATOR_INTEGRATOR_H_
#define FLIGHTLOOP_INTEGRATOR_INTEGRATOR_H_

#include <Eigen/Core>

#include "flightloop/integrator/rhs.h"

namespace flightloop {

// Advances a continuous state vector from t to t + dt under a caller-supplied
// right-hand side.
//
// The state is treated as a flat vector in R^n with Euclidean structure.
// Renormalization onto any manifold (such as Lie Group) is the caller's
// responsibility, applied after an accepted step.
//
// The RHS f(t, x, x_dot) must be a pure function with no side effects; u is
// held constant across the step (sample-and-hold).
class Integrator {
 public:
  // Convenience re-exports of the RHS contract, whose definitions live in
  // rhs.h, so integrator call sites can keep spelling Integrator::RhsStatus.
  using RhsStatus = ::flightloop::RhsStatus;
  using RhsFunction = ::flightloop::RhsFunction;

  // Result of the full outer step, where succeeding
  // advances x by dt and failing leaves x unspecified.
  // The caller is responsible for restoring x.
  enum class [[nodiscard(
      "Step may have failed, in which case, x may be "
      "unspecified and will require restoring")]] StepStatus {
    kOk,
    kFailed,
  };

  virtual StepStatus Step(double t, double dt, const RhsFunction& f,
                          Eigen::VectorXd& x) = 0;

  // Implemented on a per derived class basis for integrators
  // that require a function for clearing its own state.
  virtual void Reset() = 0;

  virtual ~Integrator();

 protected:
  Integrator() = default;
  Integrator(const Integrator&) = default;
  Integrator& operator=(const Integrator&) = default;
};

}  // namespace flightloop

#endif  // FLIGHTLOOP_INTEGRATOR_INTEGRATOR_H_
