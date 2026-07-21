// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#include "flightloop/state/state_block_view.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>

#include "flightloop/state/state_block.h"
#include "flightloop/state/state_registry.h"

namespace flightloop {
namespace {

// Layout under test: position [0,3), attitude [3,7), velocity [7,10).
class StateBlockViewTest : public ::testing::Test {
 protected:
  StateBlockViewTest() {
    const StateBlockHandle position = registry_.Register(
        StateBlockDescriptor{"position", 3, StateManifold::kEuclidean});
    const StateBlockHandle attitude = registry_.Register(
        StateBlockDescriptor{"attitude", 4, StateManifold::kUnitQuaternion});
    const StateBlockHandle velocity = registry_.Register(
        StateBlockDescriptor{"velocity", 3, StateManifold::kEuclidean});
    registry_.Finalize();

    position_ = StateBlockView(registry_, position);
    attitude_ = StateBlockView(registry_, attitude);
    velocity_ = StateBlockView(registry_, velocity);
    x_ = Eigen::VectorXd::Zero(registry_.size());
  }

  StateRegistry registry_;
  StateBlockView position_;
  StateBlockView attitude_;
  StateBlockView velocity_;
  Eigen::VectorXd x_;
};

TEST_F(StateBlockViewTest, BindsOffsetAndSizeFromTheRegistry) {
  EXPECT_EQ(position_.offset(), 0);
  EXPECT_EQ(position_.size(), 3);
  EXPECT_EQ(attitude_.offset(), 3);
  EXPECT_EQ(attitude_.size(), 4);
  EXPECT_EQ(velocity_.offset(), 7);
  EXPECT_EQ(velocity_.size(), 3);
}

TEST_F(StateBlockViewTest, ReadSeesWhatWriteStored) {
  const Eigen::Vector3d expected(4.0, 5.0, 6.0);
  position_.Write(x_) = expected;

  EXPECT_TRUE(position_.Read(x_).isApprox(expected));
}

TEST_F(StateBlockViewTest, WriteTouchesOnlyItsOwnSegment) {
  velocity_.Write(x_) = Eigen::Vector3d(1.0, 2.0, 3.0);

  EXPECT_TRUE(x_.head(7).isZero());
  EXPECT_DOUBLE_EQ(x_(7), 1.0);
  EXPECT_DOUBLE_EQ(x_(8), 2.0);
  EXPECT_DOUBLE_EQ(x_(9), 3.0);
}

// Write() is const because it mutates the vector, not the view. Views are
// meant to be held by value as cheap const members of a component.
TEST_F(StateBlockViewTest, WriteIsCallableThroughAConstView) {
  const StateBlockView view = position_;
  view.Write(x_) = Eigen::Vector3d::Constant(1.0);

  EXPECT_TRUE(x_.head(3).isApproxToConstant(1.0));
}

// The registry stores quaternions scalar-first, [w, x, y, z]. Eigen's own
// coefficient storage is scalar-last, so this ordering is precisely what a
// Map<Quaterniond> over the segment would get silently wrong. Components are
// deliberately all distinct -- an equal-valued quaternion would pass under
// every possible permutation.
TEST_F(StateBlockViewTest, WriteQuaternionStoresScalarFirst) {
  Eigen::Quaterniond q(0.1, 0.2, 0.3, 0.4);  // (w, x, y, z)
  q.normalize();

  attitude_.WriteQuaternion(x_, q);

  EXPECT_DOUBLE_EQ(x_(3), q.w());
  EXPECT_DOUBLE_EQ(x_(4), q.x());
  EXPECT_DOUBLE_EQ(x_(5), q.y());
  EXPECT_DOUBLE_EQ(x_(6), q.z());
}

TEST_F(StateBlockViewTest, ReadQuaternionInterpretsScalarFirst) {
  Eigen::Quaterniond expected(0.1, 0.2, 0.3, 0.4);
  expected.normalize();

  x_(3) = expected.w();
  x_(4) = expected.x();
  x_(5) = expected.y();
  x_(6) = expected.z();

  const Eigen::Quaterniond q = attitude_.ReadQuaternion(x_);
  EXPECT_DOUBLE_EQ(q.w(), expected.w());
  EXPECT_DOUBLE_EQ(q.x(), expected.x());
  EXPECT_DOUBLE_EQ(q.y(), expected.y());
  EXPECT_DOUBLE_EQ(q.z(), expected.z());
}

TEST_F(StateBlockViewTest, QuaternionRoundTripsThroughTheStateVector) {
  const Eigen::Quaterniond q(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));

  attitude_.WriteQuaternion(x_, q);

  EXPECT_TRUE(attitude_.ReadQuaternion(x_).isApprox(q));
}

TEST_F(StateBlockViewTest, QuaternionAccessorsRejectEuclideanBlocks) {
  EXPECT_THROW(position_.ReadQuaternion(x_), std::logic_error);
  EXPECT_THROW(position_.WriteQuaternion(x_, Eigen::Quaterniond::Identity()),
               std::logic_error);
}

TEST_F(StateBlockViewTest, VectorShorterThanTheBlockIsRejected) {
  Eigen::VectorXd too_short = Eigen::VectorXd::Zero(registry_.size() - 1);

  EXPECT_THROW(velocity_.Read(too_short), std::out_of_range);
  EXPECT_THROW(velocity_.Write(too_short), std::out_of_range);
}

TEST_F(StateBlockViewTest, QuaternionAccessorsBoundsCheckToo) {
  Eigen::VectorXd too_short = Eigen::VectorXd::Zero(5);

  EXPECT_THROW(attitude_.ReadQuaternion(too_short), std::out_of_range);
  EXPECT_THROW(
      attitude_.WriteQuaternion(too_short, Eigen::Quaterniond::Identity()),
      std::out_of_range);
}

TEST(StateBlockViewUnboundTest, DefaultConstructedViewReportsUnbound) {
  const StateBlockView view;

  EXPECT_EQ(view.offset(), -1);
  EXPECT_EQ(view.size(), 0);
}

TEST(StateBlockViewUnboundTest, DefaultConstructedViewRejectsAccess) {
  const StateBlockView view;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(10);

  EXPECT_THROW(view.Read(x), std::logic_error);
  EXPECT_THROW(view.Write(x), std::logic_error);
}

// Binding reads record(), so the registry's finalize guard is what stops a view
// from capturing an offset that later registrations would invalidate.
TEST(StateBlockViewUnboundTest, BindingRequiresAFinalizedRegistry) {
  StateRegistry registry;
  const StateBlockHandle position =
      registry.Register(StateBlockDescriptor{"position", 3});

  EXPECT_THROW(StateBlockView(registry, position), std::logic_error);
}

TEST(StateBlockViewUnboundTest, BindingRejectsDefaultConstructedHandle) {
  StateRegistry registry;
  registry.Register(StateBlockDescriptor{"position", 3});
  registry.Finalize();

  EXPECT_THROW(StateBlockView(registry, StateBlockHandle()),
               std::invalid_argument);
}

}  // namespace
}  // namespace flightloop
