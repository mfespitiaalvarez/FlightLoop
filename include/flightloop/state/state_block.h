// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#ifndef FLIGHTLOOP_STATE_STATE_BLOCK_H_
#define FLIGHTLOOP_STATE_STATE_BLOCK_H_

#include <string>

namespace flightloop {

// Sim will natively work with quaternions and users may
// need to work with quaternions in their components.
enum class StateManifold { kEuclidean, kUnitQuaternion };

// A StateBlockDescriptor is passed into a StateRegistry,
// which will return a StateBlockHandle.
struct StateBlockDescriptor {
  std::string name;
  int size = 0;
  StateManifold manifold = StateManifold::kEuclidean;
};

// A StateBlockHandle is essentially just the index (int)
// marking where a StateBlock lives inside the StateRegistry's
// vector of StateBlocks.
class StateBlockHandle {
 public:
  StateBlockHandle() = default;

 private:
  // A StateRegistry should be the only object that
  // can change the value of the offset after the
  // registration phase.
  friend class StateRegistry;

  explicit StateBlockHandle(int index) : index_(index) {}

  int index_ = -1;
};

}  // namespace flightloop
#endif  // FLIGHTLOOP_STATE_STATE_BLOCK_H_
