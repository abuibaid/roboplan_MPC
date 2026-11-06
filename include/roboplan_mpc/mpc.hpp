#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Options struct for the MPC.
struct MPCOptions {
  /// @brief The joint group name to be used by the controller.
  std::string group_name = "";

  /// @brief The time step of the prediction model, in seconds.
  double Ts = 0.1;

  /// @brief The number of steps in the prediction horizon.
  size_t N = 15;

  /// @brief Diagonal weights on the joint state error [q; dq]. Defaults to 1 for positions and 0.1
  /// for velocities if empty.
  Eigen::VectorXd Q;

  /// @brief Diagonal weights on the terminal joint state error. Defaults to 3 Q if empty.
  Eigen::VectorXd Qf;

  /// @brief Diagonal weights on the inputs (joint accelerations). Defaults to 0.01 if empty.
  Eigen::VectorXd R;

  /// @brief Diagonal weights on the input rates. Defaults to 0.1 if empty.
  Eigen::VectorXd Rd;

  /// @brief The NLP solver: "sqpmethod" for one SQP iteration per call with OSQP (real-time
  /// iteration, fast), or "ipopt" to solve to convergence on every call.
  std::string solver = "sqpmethod";

  /// @brief If true, compiles the problem functions just in time. Requires a C compiler at
  /// runtime, and takes seconds to minutes when constructing; most of the solve time is spent in
  /// IPOPT itself, so the speedup is small.
  bool jit = false;
};

/// @brief Joint-space model predictive control (MPC) for motion control.
/// @details Tracks a desired joint state while respecting the joint position, velocity,
/// acceleration, and torque limits of the scene. The joint accelerations are the inputs of a
/// double integrator, and the torques follow from the robot dynamics (RNEA). The optimal control
/// problem is formulated with multiple shooting in CasADi, and solved with one SQP iteration per
/// call (OSQP) or with IPOPT.
class MPC {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for motion control.
  /// @param options A struct containing MPC options.
  MPC(const std::shared_ptr<Scene> scene, const MPCOptions& options);

  /// @brief Sets the desired joint state.
  /// @param goal The desired joint positions, and optionally velocities (zero if empty).
  /// @throws std::invalid_argument if the sizes do not match the joint group.
  void setGoal(const JointConfiguration& goal);

  /// @brief Solves the MPC from the current joint state.
  /// @param state The current joint positions and velocities.
  /// @return The predicted joint trajectory over the horizon, starting at the current state, if
  /// the solve succeeds, otherwise an error message.
  tl::expected<JointTrajectory, std::string> solve(const JointConfiguration& state);

  /// @brief Returns the time step of the prediction model, in seconds.
  double getTimeStep() const { return Ts_; }

  /// @brief Returns the number of steps in the prediction horizon.
  size_t getHorizon() const { return N_; }

private:
  /// @brief Formulates the optimal control problem and creates the solver.
  void setupMPCProblem();

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The struct containing MPC options.
  MPCOptions options_;

  /// @brief The joint group info for the controller.
  JointGroupInfo joint_group_info_;

  /// @brief Time step, horizon, number of states [q; dq] and of inputs ddq.
  double Ts_;
  size_t N_;
  casadi_int nx_;
  casadi_int nu_;

  /// @brief Bounds on the states [q; dq], the inputs ddq, and the joint torques.
  std::vector<double> lbx_, ubx_, lbu_, ubu_, max_tau_;

  /// @brief The NLP solver.
  casadi::Function solver_;

  /// @brief The solver arguments: bounds, and the initial guess and multipliers from the
  /// previous solution (warm start).
  casadi::DMDict args_;

  /// @brief The desired joint state [q; dq].
  std::vector<double> x_desired_;

  /// @brief The previous input, to penalize the input rate of the first step.
  std::vector<double> u_prev_;

  /// @brief Whether the solver has a previous solution to warm start from.
  bool warm_start_ = false;
};

}  // namespace roboplan
