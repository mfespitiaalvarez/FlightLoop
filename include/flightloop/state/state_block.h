// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#ifndef FLIGHTLOOP_STATE_STATE_BLOCK_H_
#define FLIGHTLOOP_STATE_STATE_BLOCK_H_

#include <string>

namespace flightloop {

enum class StateManifold { kEuclidean, kUnitQuaternion };

struct StateBlockDescriptor {
  std::string name;
  int size = 0;
  StateManifold manifold = StateManifold::kEuclidean;
};

class StateBlockHandle {
 public:
  StateBlockHandle() = default;

 private:
  friend class StateRegistry;

  explicit StateBlockHandle(int index) : index_(index) {}

  int index_ = -1;
};

}  // namespace flightloop
#endif  // FLIGHTLOOP_STATE_STATE_BLOCK_H_
