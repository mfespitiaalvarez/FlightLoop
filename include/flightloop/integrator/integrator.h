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
#include <functional>

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
  // Result of one right-hand-side evaluation. Mirrors the SUNDIALS
  // CVRhsFn convention (0, ok, >0 recoverable, <0 fatal) so a
  // wrapper can forward it directly
  enum class [[nodiscard(
      "A dropped kFatalError will be "
      "silently integrated over")]] RhsStatus {
    kOk,
    kRecoverableError,
    kFatalError,
  };

  // Result of the full outer step, where succeeding
  // advances x by dt and failing leaves x unspecified.
  // The caller is responsible for restoring x.
  enum class [[nodiscard(
      "Step may have failed, in which case, x may be "
      "unspecified and will require restoring")]] StepStatus {
    kOk,
    kFailed,
  };

  // Right-hand side f of the ODE x_dot = f(t, x). Writes the derivative
  // into x_dot, which the caller preallocates to x.size(). Must be
  // side-effect-free: it is evaluated at off-trajectory stage points
  // that are thrown away
  using RhsFunction = std::function<RhsStatus(
      double t, const Eigen::VectorXd& x, Eigen::VectorXd& x_dot)>;

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
