// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#ifndef FLIGHTLOOP_STATE_STATE_VIEW_H_
#define FLIGHTLOOP_STATE_STATE_VIEW_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "flightloop/state/state_block.h"
#include "flightloop/state/state_registry.h"
namespace flightloop {

class StateBlockView {
 public:
  StateBlockView() = default;

  StateBlockView(const StateRegistry& registry, StateBlockHandle handle);

  Eigen::VectorBlock<const Eigen::VectorXd> Read(
      const Eigen::VectorXd& x) const;
  Eigen::Quaterniond ReadAsQuaternion(const Eigen::VectorXd& x) const;
  Eigen::VectorBlock<Eigen::VectorXd> Write(Eigen::VectorXd& v) const;

  int offset() const;
  int size() const;

 private:
  int offset_ = -1;
  int size_ = 0;
  StateManifold manifold_ = StateManifold::kEuclidean;
};

}  // namespace flightloop

#endif  // FLIGHTLOOP_STATE_STATE_VIEW_H_
