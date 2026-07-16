// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// 4th Order Runge-Kutta. Refer to
// https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html if you are more
// interested in reading.

#ifndef FLIGHTLOOP_INTEGRATOR_RK4_INTEGRATOR_H_
#define FLIGHTLOOP_INTEGRATOR_RK4_INTEGRATOR_H_

#include <Eigen/Core>

#include "flightloop/integrator/integrator.h"
#include "flightloop/ode/rhs.h"

namespace flightloop {

class Rk4Integrator final : public Integrator {
 public:
  explicit Rk4Integrator(Eigen::Index n);
  StepStatus Step(double t, double dt, const RhsFunction& rhs,
                  Eigen::VectorXd& x) override;

  void Reset() override {}

 private:
  Eigen::VectorXd k1_;
  Eigen::VectorXd k2_;
  Eigen::VectorXd k3_;
  Eigen::VectorXd k4_;
  Eigen::VectorXd x_stage_;
};

}  // namespace flightloop

#endif  // FLIGHTLOOP_INTEGRATOR_RK4_INTEGRATOR_H_
