// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// A fixed list of reference frame ids to be used for
// pose conversions and tracking

#ifndef FLIGHTLOOP_FRAMES_REFERENCE_FRAMES_H_
#define FLIGHTLOOP_FRAMES_REFERENCE_FRAMES_H_

namespace flightloop {

enum class ReferenceFrame { kEci, kEcef, kNed, kBody, kWind };

}  // namespace flightloop
#endif  // FLIGHTLOOP_FRAMES_REFERENCE_FRAMES_H_
