#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the MPC.
/// @param m The nanobind core module.
void initMpc(nanobind::module_& m);

}  // namespace roboplan
