// Copyright (c) 2026 mfespitiaalvarez
//
// SPDX-License-Identifier: MIT

// Out-of-line anchor for the Integrator interface. Defining a single non-inline
// virtual here (the destructor) fixes Integrator's key function to this
// translation unit, so its vtable and typeinfo are emitted once, here, rather
// than as weak symbols in every translation unit that derives from or uses it.

#include "flightloop/integrator/integrator.h"

namespace flightloop {

Integrator::~Integrator() = default;

}  // namespace flightloop
