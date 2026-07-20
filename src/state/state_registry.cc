// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/state/state_registry.h"

#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "flightloop/state/state_block.h"

namespace flightloop {

StateBlockHandle StateRegistry::Register(StateBlockDescriptor descriptor) {
  if (finalized_) {
    throw std::logic_error("StateRegistry: Register() called after Finalize()");
  }

  if (descriptor.size <= 0) {
    throw std::invalid_argument(
        "StateRegistry: Register() passed in "
        "StateBlockDescriptor with size <= 0.");
  }

  if (descriptor.name.empty()) {
    throw std::invalid_argument(
        "StateRegistry: Register() passed in "
        "StateBlockDescriptor with empty name.");
  }

  if (Find(descriptor.name)) {
    throw std::invalid_argument("StateRegistry: duplicate block name '" +
                                descriptor.name + "'.");
  }

  if (descriptor.manifold == StateManifold::kUnitQuaternion &&
      descriptor.size != 4) {
    throw std::invalid_argument("StateRegistry: Unit quaternion block '" +
                                descriptor.name + "' must have size 4.");
  }

  blocks_.push_back(StateBlockRecord{std::move(descriptor)});
  return StateBlockHandle(static_cast<int>(blocks_.size()) - 1);
}

void StateRegistry::Finalize() {
  if (finalized_) {
    throw std::logic_error(
        "StateRegistry: Finalize() called when already finalized");
  }
  int running_offset = 0;
  for (auto& block : blocks_) {
    block.offset = running_offset;
    running_offset += block.descriptor.size;
  }
  size_ = running_offset;
  finalized_ = true;
}

bool StateRegistry::finalized() const { return finalized_; }

int StateRegistry::size() const {
  if (!finalized_) {
    throw std::logic_error(
        "StateRegistry: Accessing size before finalized registry");
  }
  return size_;
}

const StateBlockRecord& StateRegistry::record(StateBlockHandle handle) const {
  if (!finalized_) {
    throw std::logic_error(
        "StateRegistry: record() called before finalized registry.");
  }

  if (handle.index_ < 0 || handle.index_ >= static_cast<int>(blocks_.size())) {
    throw std::invalid_argument(
        "StateRegistry: record() passed a handle that does not refer to a "
        "registered block. Did you pass a default constructed handle? "
        "(index_ = -1)");
  }

  return blocks_[handle.index_];
}

const std::vector<StateBlockRecord>& StateRegistry::blocks() const {
  if (!finalized_) {
    throw std::logic_error(
        "StateRegistry: Attempting to access blocks before finalized registry");
  }

  return blocks_;
}

std::optional<StateBlockHandle> StateRegistry::Find(
    std::string_view name) const {
  for (int index = 0; const auto& block : blocks_) {
    if (block.descriptor.name == name) {
      return StateBlockHandle(index);
    }
    ++index;
  }
  return std::nullopt;
}

}  // namespace flightloop
