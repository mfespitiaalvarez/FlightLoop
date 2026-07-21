// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

#ifndef FLIGHTLOOP_SIGNALS_SIGNAL_H_
#define FLIGHTLOOP_SIGNALS_SIGNAL_H_

#include <memory>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace flightloop {

using ComponentId = int;
using TypeKey = const char*;

template <class T>
struct SignalType {
  inline static const char id = 0;
};

template <class T>
inline constexpr TypeKey kTypeKey = &SignalType<T>::id;

template <class T>
struct SignalDefault {
  static T value() { return T{}; }
};

template <class S, int R, int C, int O, int MR, int MC>
struct SignalDefault<Eigen::Matrix<S, R, C, O, MR, MC>> {
  static Eigen::Matrix<S, R, C, O, MR, MC> value() {
    return Eigen::Matrix<S, R, C, O, MR, MC>::Zero();
  }
};

template <>
struct SignalDefault<Eigen::Quaterniond> {
  static Eigen::Quaterniond value() { return Eigen::Quaterniond::Identity(); }
};

class OutputHandle {
 public:
  OutputHandle() = default;

 private:
  friend class SignalRegistry;

  explicit OutputHandle(int index) : index_(index) {}

  int index_ = -1;
};

class InputHandle {
 public:
  InputHandle() = default;

 private:
  friend class SignalRegistry;

  explicit InputHandle(int index) : index_(index) {}

  int index_ = -1;
};

struct SignalValueBase {
  virtual ~SignalValueBase() = default;

  virtual std::unique_ptr<SignalValueBase> Clone() const = 0;
};

template <class T>
struct SignalValue : SignalValueBase {
  T value = SignalDefault<T>::value();
  std::unique_ptr<SignalValueBase> Clone() const override {
    return std::make_unique<SignalValue>(*this);
  }
};

}  // namespace flightloop
#endif  // FLIGHTLOOP_SIGNALS_SIGNAL_H_
