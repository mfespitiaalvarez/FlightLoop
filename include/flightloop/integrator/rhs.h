// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// The right-hand-side contract of the ODE x_dot = f(t, x): the vocabulary
// shared between code that produces derivatives (force models, vehicle
// components) and the integrators that consume them. Kept separate from
// integrator.h so derivative producers do not depend on the time-stepping
// interface.

#ifndef FLIGHTLOOP_INTEGRATOR_RHS_H_
#define FLIGHTLOOP_INTEGRATOR_RHS_H_

#include <Eigen/Core>
#include <functional>

namespace flightloop {

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

// Right-hand side f of the ODE x_dot = f(t, x). Writes the derivative
// into x_dot, which the caller preallocates to x.size(). Must be
// side-effect-free: it is evaluated at off-trajectory stage points
// that are thrown away
using RhsFunction = std::function<RhsStatus(
    double t, const Eigen::VectorXd& x, Eigen::VectorXd& x_dot)>;

}  // namespace flightloop

#endif  // FLIGHTLOOP_INTEGRATOR_RHS_H_
