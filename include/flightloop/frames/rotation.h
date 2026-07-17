// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// Rotation conversions and definitions specific for aerospace applications.

#ifndef FLIGHTLOOP_FRAMES_ROTATION_H_
#define FLIGHTLOOP_FRAMES_ROTATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numbers>

namespace flightloop {

// Rotation conventions
//
// R_AB (or q_AB) re-expresses a vector's frame-B coordinates in frame A:
//   v_A = R_AB * v_B
// Adjacent subscripts cancel when composing: R_AC = R_AB * R_BC.
// This is the passive (coordinate re-expression) reading. Note it is the
// transpose/conjugate of the classical flight-dynamics DCM C_b/n
// (Stevens & Lewis), which maps NED -> body; transpose before comparing
// element formulas against such references.
//
// Euler angles use the intrinsic aerospace 3-2-1 sequence (yaw psi about z,
// then pitch theta about the new y, then roll phi about the newest x):
//   R_NB = Rz(yaw) * Ry(pitch) * Rx(roll)
// All angles are in radians. Quaternions follow Eigen's Hamilton convention
// and are assumed unit-norm.
//
// Gimbal lock: at pitch = +/-pi/2 only the difference (yaw -+ roll) is
// observable; extraction returns roll = 0 with everything absorbed into yaw.

struct Euler321 {
  double yaw;    // psi, rad, [-pi, pi]
  double pitch;  // theta, rad, [-pi/2, pi/2]
  double roll;   // phi, rad, [-pi, pi]
};

// Returns q_NB, the attitude of body frame B relative to NED frame N, such
// that v_N = q_NB * v_B.
inline Eigen::Quaterniond QuaternionFromEuler321(const Euler321& e) {
  return Eigen::AngleAxisd(e.yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(e.pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(e.roll, Eigen::Vector3d::UnitX());
}

// Extracts the 3-2-1 Euler angles from R = R_NB; inverse of
// QuaternionFromEuler321 under the gimbal-lock convention above.
inline Euler321 Euler321FromDcm(const Eigen::Matrix3d& R) {
  const double sin_pitch = -R(2, 0);
  const double cos_pitch = std::hypot(R(2, 1), R(2, 2));

  if (cos_pitch < 1e-9) {
    return {std::atan2(-R(0, 1), R(1, 1)),
            std::copysign(std::numbers::pi / 2, sin_pitch), 0.0};
  }
  return {std::atan2(R(1, 0), R(0, 0)), std::atan2(sin_pitch, cos_pitch),
          std::atan2(R(2, 1), R(2, 2))};
}

// Extracts the 3-2-1 Euler angles from q_NB. Assumes |q| = 1; convert
// near-unit quaternions with q.normalized() first if drift matters.
inline Euler321 Euler321FromQuaternion(const Eigen::Quaterniond& q) {
  return Euler321FromDcm(q.toRotationMatrix());
}

}  // namespace flightloop

#endif  // FLIGHTLOOP_FRAMES_ROTATION_H_
