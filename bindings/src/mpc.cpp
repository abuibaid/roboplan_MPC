#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_mpc/mpc.hpp>

#include <roboplan_bindings/expected.hpp>
#include <roboplan_mpc_bindings/mpc.hpp>

namespace roboplan {

using namespace nanobind::literals;

void initMpc(nanobind::module_& m) {
  nanobind::class_<MPCOptions>(m, "MPCOptions", "Options struct for the MPC.")
      .def(nanobind::init<const std::string&, double, size_t, const Eigen::VectorXd&,
                          const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
                          const std::string&, bool>(),
           "group_name"_a = "", "Ts"_a = 0.1, "N"_a = 15, "Q"_a = Eigen::VectorXd(),
           "Qf"_a = Eigen::VectorXd(), "R"_a = Eigen::VectorXd(), "Rd"_a = Eigen::VectorXd(),
           "solver"_a = "sqpmethod", "jit"_a = false)
      .def_rw("group_name", &MPCOptions::group_name,
              "The joint group name to be used by the controller.")
      .def_rw("Ts", &MPCOptions::Ts, "The time step of the prediction model, in seconds.")
      .def_rw("N", &MPCOptions::N, "The number of steps in the prediction horizon.")
      .def_rw("Q", &MPCOptions::Q,
              "Diagonal weights on the joint state error [q; dq]. Defaults to 1 for positions "
              "and 0.1 for velocities if empty.")
      .def_rw("Qf", &MPCOptions::Qf,
              "Diagonal weights on the terminal joint state error. Defaults to 3 Q if empty.")
      .def_rw("R", &MPCOptions::R,
              "Diagonal weights on the inputs (joint accelerations). Defaults to 0.01 if empty.")
      .def_rw("Rd", &MPCOptions::Rd,
              "Diagonal weights on the input rates. Defaults to 0.1 if empty.")
      .def_rw("solver", &MPCOptions::solver,
              "The NLP solver: 'sqpmethod' for one SQP iteration per call with OSQP (real-time "
              "iteration, fast), or 'ipopt' to solve to convergence on every call.")
      .def_rw("jit", &MPCOptions::jit,
              "If true, compiles the problem functions just in time. Requires a C compiler at "
              "runtime, and takes seconds to minutes when constructing; most of the solve time "
              "is spent in IPOPT itself, so the speedup is small.");

  nanobind::class_<MPC>(m, "MPC", "Joint-space model predictive control (MPC) for motion control.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const MPCOptions&>(), "scene"_a,
           "options"_a)
      .def("setGoal", &MPC::setGoal, "Sets the desired joint state.", "goal"_a)
      .def("solve", unwrap_expected(&MPC::solve),
           nanobind::call_guard<nanobind::gil_scoped_release>(),
           "Solves the MPC from the current joint state. Returns the predicted joint trajectory "
           "over the horizon, starting at the current state.",
           "state"_a)
      .def("getTimeStep", &MPC::getTimeStep,
           "Returns the time step of the prediction model, in seconds.")
      .def("getHorizon", &MPC::getHorizon,
           "Returns the number of steps in the prediction horizon.");
}

}  // namespace roboplan
