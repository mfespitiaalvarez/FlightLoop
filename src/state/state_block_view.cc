// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/state/state_block_view.h"

#include <stdexcept>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "flightloop/state/state_block.h"
#include "flightloop/state/state_registry.h"

namespace flightloop {
namespace {

void CheckBounds(int offset, int size, Eigen::Index vector_size) {
  if (offset < 0) {
    throw std::logic_error(
        "StateBlockView: used before being bound to a block.");
  }
  if (offset + size > vector_size) {
    throw std::out_of_range(
        "StateBlockView: block extends past end of vector.");
  }
}

}  // namespace

StateBlockView::StateBlockView(const StateRegistry& registry,
                               StateBlockHandle handle) {
  const StateBlockRecord& record = registry.record(handle);
  offset_ = record.offset;
  size_ = record.descriptor.size;
  manifold_ = record.descriptor.manifold;
}

// StateBlockView is the only place to touch the registry.
Eigen::VectorBlock<const Eigen::VectorXd> StateBlockView::Read(
    const Eigen::VectorXd& x) const {
  CheckBounds(offset_, size_, x.size());
  return x.segment(offset_, size_);
}

// The registry stores quaternions in JPL, scalar-first [w,x,y,z] convention.
// Eigen's internal storage/mathematical convention is Hamilton, so
// Map<Quaterniond> would fail silently.
Eigen::Quaterniond StateBlockView::ReadQuaternion(
    const Eigen::VectorXd& x) const {
  if (manifold_ != StateManifold::kUnitQuaternion) {
    throw std::logic_error(
        "StateBlockView: ReadQuaternion() on a non-quaternion block.");
  }
  CheckBounds(offset_, 4, x.size());
  const auto q = x.segment(offset_, 4);
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);  // (w, x, y, z)
}

Eigen::VectorBlock<Eigen::VectorXd> StateBlockView::Write(
    Eigen::VectorXd& v) const {
  CheckBounds(offset_, size_, v.size());
  return v.segment(offset_, size_);
}

void StateBlockView::WriteQuaternion(Eigen::VectorXd& v,
                                     const Eigen::Quaterniond& q) const {
  if (manifold_ != StateManifold::kUnitQuaternion) {
    throw std::logic_error(
        "StateBlockView: WriteQuaternion() on a non-quaternion block.");
  }
  CheckBounds(offset_, 4, v.size());
  auto seg = v.segment(offset_, 4);
  seg[0] = q.w();
  seg[1] = q.x();
  seg[2] = q.y();
  seg[3] = q.z();
}

int StateBlockView::offset() const { return offset_; }
int StateBlockView::size() const { return size_; }

}  // namespace flightloop
