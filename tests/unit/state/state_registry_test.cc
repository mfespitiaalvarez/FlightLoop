// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/state/state_registry.h"

#include <gtest/gtest.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "flightloop/state/state_block.h"

namespace flightloop {
namespace {

StateBlockDescriptor Block(std::string name, int size,
                           StateManifold manifold = StateManifold::kEuclidean) {
  return StateBlockDescriptor{std::move(name), size, manifold};
}

TEST(StateRegistryTest, PacksBlocksContiguouslyInRegistrationOrder) {
  StateRegistry registry;
  const StateBlockHandle position = registry.Register(Block("position", 3));
  const StateBlockHandle attitude =
      registry.Register(Block("attitude", 4, StateManifold::kUnitQuaternion));
  const StateBlockHandle velocity = registry.Register(Block("velocity", 3));
  registry.Finalize();

  EXPECT_EQ(registry.record(position).offset, 0);
  EXPECT_EQ(registry.record(attitude).offset, 3);
  EXPECT_EQ(registry.record(velocity).offset, 7);
  EXPECT_EQ(registry.size(), 10);
}

TEST(StateRegistryTest, RecordCarriesTheDescriptorThrough) {
  StateRegistry registry;
  const StateBlockHandle attitude =
      registry.Register(Block("attitude", 4, StateManifold::kUnitQuaternion));
  registry.Finalize();

  const StateBlockRecord& record = registry.record(attitude);
  EXPECT_EQ(record.descriptor.name, "attitude");
  EXPECT_EQ(record.descriptor.size, 4);
  EXPECT_EQ(record.descriptor.manifold, StateManifold::kUnitQuaternion);
}

TEST(StateRegistryTest, BlocksAreExposedInRegistrationOrder) {
  StateRegistry registry;
  registry.Register(Block("first", 1));
  registry.Register(Block("second", 2));
  registry.Finalize();

  const std::vector<StateBlockRecord>& blocks = registry.blocks();
  ASSERT_EQ(blocks.size(), 2u);
  EXPECT_EQ(blocks[0].descriptor.name, "first");
  EXPECT_EQ(blocks[1].descriptor.name, "second");
}

TEST(StateRegistryTest, EmptyRegistryFinalizesToSizeZero) {
  StateRegistry registry;
  registry.Finalize();

  EXPECT_EQ(registry.size(), 0);
  EXPECT_TRUE(registry.blocks().empty());
}

TEST(StateRegistryTest, FinalizedFlagTracksLifecycle) {
  StateRegistry registry;
  EXPECT_FALSE(registry.finalized());
  registry.Finalize();
  EXPECT_TRUE(registry.finalized());
}

TEST(StateRegistryTest, FindLocatesRegisteredBlocksByName) {
  StateRegistry registry;
  registry.Register(Block("position", 3));
  registry.Register(Block("velocity", 3));
  registry.Finalize();

  const std::optional<StateBlockHandle> velocity = registry.Find("velocity");
  ASSERT_TRUE(velocity.has_value());
  EXPECT_EQ(registry.record(*velocity).descriptor.name, "velocity");
  EXPECT_EQ(registry.record(*velocity).offset, 3);

  EXPECT_FALSE(registry.Find("no_such_block").has_value());
}

// Find() is the one query with no finalize guard, and it has to stay that way:
// Register() calls it to reject duplicate names, so it runs mid-registration.
TEST(StateRegistryTest, FindWorksBeforeFinalize) {
  StateRegistry registry;
  registry.Register(Block("position", 3));

  EXPECT_TRUE(registry.Find("position").has_value());
  EXPECT_FALSE(registry.Find("velocity").has_value());
}

TEST(StateRegistryTest, RegisterRejectsNonPositiveSize) {
  StateRegistry registry;
  EXPECT_THROW(registry.Register(Block("zero", 0)), std::invalid_argument);
  EXPECT_THROW(registry.Register(Block("negative", -1)), std::invalid_argument);
}

TEST(StateRegistryTest, RegisterRejectsEmptyName) {
  StateRegistry registry;
  EXPECT_THROW(registry.Register(Block("", 3)), std::invalid_argument);
}

TEST(StateRegistryTest, RegisterRejectsDuplicateName) {
  StateRegistry registry;
  registry.Register(Block("position", 3));
  EXPECT_THROW(registry.Register(Block("position", 4)), std::invalid_argument);
}

// Names key persistence, so a rejected registration must leave no partial block
// behind -- a half-registered name would silently shift every later offset.
TEST(StateRegistryTest, RejectedRegistrationsDoNotAffectLayout) {
  StateRegistry registry;
  registry.Register(Block("position", 3));
  EXPECT_THROW(registry.Register(Block("position", 4)), std::invalid_argument);
  EXPECT_THROW(registry.Register(Block("bad_size", 0)), std::invalid_argument);
  EXPECT_THROW(registry.Register(Block("", 3)), std::invalid_argument);
  registry.Finalize();

  EXPECT_EQ(registry.size(), 3);
  EXPECT_EQ(registry.blocks().size(), 1u);
}

TEST(StateRegistryTest, QuaternionBlockMustHaveSizeFour) {
  StateRegistry registry;
  EXPECT_THROW(registry.Register(Block("q", 3, StateManifold::kUnitQuaternion)),
               std::invalid_argument);
  EXPECT_THROW(registry.Register(Block("q", 5, StateManifold::kUnitQuaternion)),
               std::invalid_argument);
  EXPECT_NO_THROW(
      registry.Register(Block("q", 4, StateManifold::kUnitQuaternion)));
}

TEST(StateRegistryTest, RegisterAfterFinalizeThrows) {
  StateRegistry registry;
  registry.Finalize();
  EXPECT_THROW(registry.Register(Block("late", 3)), std::logic_error);
}

TEST(StateRegistryTest, FinalizeTwiceThrows) {
  StateRegistry registry;
  registry.Finalize();
  EXPECT_THROW(registry.Finalize(), std::logic_error);
}

TEST(StateRegistryTest, LayoutQueriesBeforeFinalizeThrow) {
  StateRegistry registry;
  const StateBlockHandle position = registry.Register(Block("position", 3));

  EXPECT_THROW(registry.size(), std::logic_error);
  EXPECT_THROW(registry.record(position), std::logic_error);
  EXPECT_THROW(registry.blocks(), std::logic_error);
}

TEST(StateRegistryTest, RecordRejectsDefaultConstructedHandle) {
  StateRegistry registry;
  registry.Register(Block("position", 3));
  registry.Finalize();

  EXPECT_THROW(registry.record(StateBlockHandle()), std::invalid_argument);
}

}  // namespace
}  // namespace flightloop
