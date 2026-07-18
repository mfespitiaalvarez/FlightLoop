// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#ifndef FLIGHTLOOP_STATE_STATE_REGISTRY_H_
#define FLIGHTLOOP_STATE_STATE_REGISTRY_H_

#include <optional>
#include <string_view>
#include <vector>

#include "flightloop/state/state_block.h"

namespace flightloop {

struct StateBlockRecord {
  StateBlockDescriptor descriptor;
  int offset = -1;
};

class StateRegistry {
 public:
  StateRegistry() = default;

  StateRegistry(const StateRegistry&) = delete;
  StateRegistry& operator=(const StateRegistry&) = delete;

  StateBlockHandle Register(StateBlockDescriptor descriptor);
  void Finalize();

  bool finalized() const;
  int size() const;
  int offset(StateBlockHandle handle) const;
  int block_size(StateBlockHandle handle) const;

  const std::vector<StateBlockRecord>& blocks() const;

  std::optional<StateBlockHandle> Find(std::string_view name) const;

 private:
  std::vector<StateBlockRecord> blocks_;
  int size_ = 0;
  bool finalized_ = false;
};

}  // namespace flightloop

#endif  // FLIGHTLOOP_STATE_STATE_REGISTRY_H_
