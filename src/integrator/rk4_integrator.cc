// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/integrator/rk4_integrator.h"

#include <Eigen/Core>
#include <cassert>
#include <stdexcept>

#include "flightloop/integrator/integrator.h"
#include "flightloop/ode/rhs.h"

namespace flightloop {

namespace {
Eigen::Index ValidateSize(Eigen::Index n) {
  if (n <= 0) {
    throw std::invalid_argument("Rk4Integrator: state size must be positive");
  }
  return n;
}

}  // namespace

Rk4Integrator::Rk4Integrator(Eigen::Index n)
    : k1_(ValidateSize(n)), k2_(n), k3_(n), k4_(n), x_stage_(n) {}

Integrator::StepStatus Rk4Integrator::Step(double t, double dt,
                                           const RhsFunction& f,
                                           Eigen::VectorXd& x) {
  assert(x.size() == k1_.size() &&
         "Rk4Integrator: x.size() must match the size passed at construction");
  if (f(t, x, k1_) != RhsStatus::kOk) return StepStatus::kFailed;
  x_stage_ = x + k1_ * dt / 2;
  if (f(t + dt / 2, x_stage_, k2_) != RhsStatus::kOk)
    return StepStatus::kFailed;
  x_stage_ = x + k2_ * dt / 2;
  if (f(t + dt / 2, x_stage_, k3_) != RhsStatus::kOk)
    return StepStatus::kFailed;
  x_stage_ = x + k3_ * dt;
  if (f(t + dt, x_stage_, k4_) != RhsStatus::kOk) return StepStatus::kFailed;

  x = x + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6;
  return StepStatus::kOk;
}

}  // namespace flightloop
