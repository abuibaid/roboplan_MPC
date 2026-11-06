#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/autodiff/casadi.hpp>

#include <roboplan_mpc/mpc.hpp>

namespace roboplan {

using casadi::DM;
using casadi::SX;

namespace {

/// @brief Repeats a vector n times.
std::vector<double> repeat(const std::vector<double>& v, size_t n) {
  std::vector<double> out;
  for (size_t i = 0; i < n; ++i) {
    out.insert(out.end(), v.begin(), v.end());
  }
  return out;
}

std::vector<double> toStd(const Eigen::VectorXd& v) { return {v.data(), v.data() + v.size()}; }

}  // namespace

MPC::MPC(const std::shared_ptr<Scene> scene, const MPCOptions& options)
    : scene_{scene}, options_{options}, Ts_{options.Ts}, N_{options.N} {
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize MPC: " + maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();
  if (joint_group_info_.has_continuous_dofs) {
    throw std::runtime_error("Could not initialize MPC: continuous joints are not supported.");
  }
  nu_ = static_cast<casadi_int>(joint_group_info_.v_indices.size());  // num of inputs
  nx_ = 2 * nu_;                                                      // system order

  // Boundaries on the states [q; dq], the inputs ddq, and the torques, from the scene
  const auto [lower_q, upper_q] = scene_->getPositionLimitVectors(options.group_name).value();
  const auto [lower_dq, upper_dq] = scene_->getVelocityLimitVectors(options.group_name).value();
  const auto [lower_ddq, upper_ddq] =
      scene_->getAccelerationLimitVectors(options.group_name).value();
  Eigen::VectorXd max_tau(nu_);
  for (casadi_int i = 0; i < nu_; ++i) {
    max_tau[i] = scene_->getModel().effortLimit[joint_group_info_.v_indices[i]];
  }
  lbx_ = toStd((Eigen::VectorXd(nx_) << lower_q, lower_dq).finished());
  ubx_ = toStd((Eigen::VectorXd(nx_) << upper_q, upper_dq).finished());
  lbu_ = toStd(lower_ddq);
  ubu_ = toStd(upper_ddq);
  max_tau_ = toStd(max_tau);

  x_desired_ = std::vector<double>(nx_, 0.0);
  u_prev_ = std::vector<double>(nu_, 0.0);
  setupMPCProblem();
}

void MPC::setupMPCProblem() {
  const auto N = static_cast<casadi_int>(N_);
  const auto n = nu_;

  // ###### Weights ######
  const auto diag = [](const Eigen::VectorXd& w) { return DM::diag(DM(toStd(w))); };
  Eigen::VectorXd Qq = options_.Q;
  if (Qq.size() == 0) {
    Qq = (Eigen::VectorXd(nx_) << Eigen::VectorXd::Ones(n), 0.1 * Eigen::VectorXd::Ones(n))
             .finished();
  }
  const DM Q = diag(Qq);
  const DM Qf = diag(options_.Qf.size() > 0 ? options_.Qf : Eigen::VectorXd(3 * Qq));
  const DM R = diag(options_.R.size() > 0 ? options_.R : Eigen::VectorXd::Constant(n, 0.01));
  const DM Rd = diag(options_.Rd.size() > 0 ? options_.Rd : Eigen::VectorXd::Constant(n, 0.1));

  // ###### System dynamics ######
  // Double integrator with the joint accelerations as inputs, x = [q; dq], u = ddq,
  // discretized exactly with a zero-order hold:
  //   x_k+1 = Ad x_k + Bd u_k,  Ad = [I, Ts I; 0, I],  Bd = [Ts^2/2 I; Ts I]
  const DM I = DM::eye(n);
  const DM Ad = DM::vertcat({DM::horzcat({I, Ts_ * I}), DM::horzcat({DM::zeros(n, n), I})});
  const DM Bd = DM::vertcat({0.5 * Ts_ * Ts_ * I, Ts_ * I});

  // Joint torques from the robot dynamics (RNEA): tau = M(q) ddq + C(q, dq) dq + g(q).
  // Joints outside the group are held at their current positions, at rest.
  using ADScalar = SX;
  const pinocchio::ModelTpl<ADScalar> ad_model = scene_->getModel().cast<ADScalar>();
  pinocchio::DataTpl<ADScalar> ad_data(ad_model);
  const SX q_sym = SX::sym("q", n);
  const SX dq_sym = SX::sym("dq", n);
  const SX ddq_sym = SX::sym("ddq", n);
  const Eigen::VectorXd q_current = scene_->getCurrentJointPositions();
  Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> q_full(ad_model.nq), v_full(ad_model.nv),
      a_full(ad_model.nv);
  for (Eigen::Index i = 0; i < ad_model.nq; ++i) {
    q_full[i] = q_current[i];
  }
  for (Eigen::Index i = 0; i < ad_model.nv; ++i) {
    v_full[i] = 0.0;
    a_full[i] = 0.0;
  }
  for (casadi_int i = 0; i < n; ++i) {
    q_full[joint_group_info_.q_indices[i]] = q_sym(i);
    v_full[joint_group_info_.v_indices[i]] = dq_sym(i);
    a_full[joint_group_info_.v_indices[i]] = ddq_sym(i);
  }
  pinocchio::rnea(ad_model, ad_data, q_full, v_full, a_full);
  SX tau_sym = SX::zeros(n);
  for (casadi_int i = 0; i < n; ++i) {
    tau_sym(i) = ad_data.tau[joint_group_info_.v_indices[i]];
  }
  const casadi::Function torque("torque", {q_sym, dq_sym, ddq_sym}, {tau_sym});

  // ######## Problem formalization ###
  // ######## multiple shooting, NLP ##
  const SX X = SX::sym("X", nx_, N + 1);
  const SX U = SX::sym("U", nu_, N);

  // ###### Parameters ######
  // P = [initial state; desired state; previous input]
  const SX P = SX::sym("P", nx_ + nx_ + nu_);
  const SX x_initial = P(casadi::Slice(0, nx_));
  const SX x_desired = P(casadi::Slice(nx_, 2 * nx_));
  const SX u_prev = P(casadi::Slice(2 * nx_, 2 * nx_ + nu_));

  // Formulate the objective function
  //   J = sum_j (x_d - x_j)' Q (x_d - x_j) + u_j' R u_j + (u_j - u_j-1)' Rd (u_j - u_j-1)
  //       + (x_d - x_N)' Qf (x_d - x_N)
  // with the dynamics as equality constraints:  0 <= x_j+1 - (Ad x_j + Bd u_j) <= 0
  SX obj = 0;
  std::vector<SX> g = {X(casadi::Slice(), 0) - x_initial};
  std::vector<SX> g_tau;
  for (casadi_int j = 0; j < N; ++j) {
    const SX x = X(casadi::Slice(), j);
    const SX x_next = X(casadi::Slice(), j + 1);
    const SX u = U(casadi::Slice(), j);
    const SX u_old = (j == 0) ? u_prev : SX(U(casadi::Slice(), j - 1));
    obj += SX::mtimes({(x_desired - x).T(), Q, x_desired - x}) + SX::mtimes({u.T(), R, u}) +
           SX::mtimes({(u - u_old).T(), Rd, u - u_old});
    g.push_back(x_next - (SX::mtimes(Ad, x) + SX::mtimes(Bd, u)));

    // Torque limits:  -tau_max <= tau(q_j, dq_j, ddq_j) <= tau_max
    g_tau.push_back(
        torque(std::vector<SX>{x(casadi::Slice(0, n)), x(casadi::Slice(n, nx_)), u}).at(0));
  }
  // Add the terminal cost
  obj +=
      SX::mtimes({(x_desired - X(casadi::Slice(), N)).T(), Qf, x_desired - X(casadi::Slice(), N)});
  g.insert(g.end(), g_tau.begin(), g_tau.end());

  const SX opt_var = SX::vertcat({SX::reshape(X, nx_ * (N + 1), 1), SX::reshape(U, nu_ * N, 1)});
  const casadi::SXDict nlp_prob = {{"f", obj}, {"x", opt_var}, {"g", SX::vertcat(g)}, {"p", P}};

  // Equality constraints (dynamics) and inequality constraints (torques)
  std::vector<double> lbg(nx_ * (N + 1), 0.0);
  std::vector<double> ubg(nx_ * (N + 1), 0.0);
  for (casadi_int j = 0; j < N; ++j) {
    for (const double tau_max : max_tau_) {
      lbg.push_back(-tau_max);
      ubg.push_back(tau_max);
    }
  }
  args_["lbg"] = lbg;
  args_["ubg"] = ubg;

  // Boundaries constraints in states and controls. The initial state is not bounded: it is fixed
  // by the equality constraint, and a measured state slightly outside the bounds would otherwise
  // make the problem infeasible.
  auto lbx = repeat(lbx_, N + 1);
  auto ubx = repeat(ubx_, N + 1);
  std::fill_n(lbx.begin(), nx_, -casadi::inf);
  std::fill_n(ubx.begin(), nx_, casadi::inf);
  const auto lbu = repeat(lbu_, N);
  const auto ubu = repeat(ubu_, N);
  lbx.insert(lbx.end(), lbu.begin(), lbu.end());
  ubx.insert(ubx.end(), ubu.begin(), ubu.end());
  args_["lbx"] = lbx;
  args_["ubx"] = ubx;

  // ###### Solver ######
  if (options_.solver == "sqpmethod") {
    // One SQP iteration per control cycle (real-time iteration), warm started from the previous
    // solution. The Hessian is the one of the quadratic objective only (Gauss-Newton), ignoring
    // the curvature of the torque constraints, which keeps every QP convex.
    const SX lam_f = SX::sym("lam_f");
    const SX lam_g = SX::sym("lam_g", nlp_prob.at("g").size1());
    const casadi::Function hess_lag("nlp_hess_l", {opt_var, P, lam_f, lam_g},
                                    {lam_f * SX::hessian(obj, opt_var)},
                                    {"x", "p", "lam_f", "lam_g"}, {"hess_gamma_x_x"});
    const casadi::Dict osqp_options = {{"verbose", false},
                                       {"eps_abs", 1e-5},
                                       {"eps_rel", 1e-5},
                                       {"max_iter", 4000},
                                       {"polish", false}};
    const casadi::Dict options = {
        {"qpsol", "osqp"},
        {"qpsol_options", casadi::Dict{{"osqp", osqp_options}, {"error_on_fail", false}}},
        {"hess_lag", hess_lag},
        {"max_iter", 1},
        {"print_time", false},
        {"print_header", false},
        {"print_iteration", false},
        {"print_status", false},
        {"error_on_fail", false},
        {"jit", options_.jit}};
    solver_ = casadi::nlpsol("solver", "sqpmethod", nlp_prob, options);
  } else if (options_.solver == "ipopt") {
    const casadi::Dict ipopt_options = {{"max_iter", 1000},
                                        {"print_level", 0},
                                        {"sb", "yes"},
                                        {"acceptable_tol", 1e-4},
                                        {"acceptable_obj_change_tol", 1e-6},
                                        {"linear_solver", "mumps"},
                                        {"warm_start_init_point", "yes"},
                                        {"fast_step_computation", "yes"}};
    const casadi::Dict options = {
        {"print_time", false}, {"jit", options_.jit}, {"ipopt", ipopt_options}};
    solver_ = casadi::nlpsol("solver", "ipopt", nlp_prob, options);
  } else {
    throw std::runtime_error("Could not initialize MPC: unknown solver '" + options_.solver +
                             "', expected 'sqpmethod' or 'ipopt'.");
  }
}

void MPC::setGoal(const JointConfiguration& goal) {
  if (goal.positions.size() != nu_ ||
      (goal.velocities.size() > 0 && goal.velocities.size() != nu_)) {
    throw std::invalid_argument("The goal must have " + std::to_string(nu_) +
                                " joint positions, and optionally velocities.");
  }
  x_desired_ = toStd(
      (Eigen::VectorXd(nx_) << goal.positions,
       goal.velocities.size() > 0 ? goal.velocities : Eigen::VectorXd(Eigen::VectorXd::Zero(nu_)))
          .finished());
}

tl::expected<JointTrajectory, std::string> MPC::solve(const JointConfiguration& state) {
  if (state.positions.size() != nu_ || state.velocities.size() != nu_) {
    return tl::make_unexpected("The state must have " + std::to_string(nu_) +
                               " joint positions and velocities.");
  }
  const auto N = static_cast<casadi_int>(N_);
  const auto x0 = toStd((Eigen::VectorXd(nx_) << state.positions, state.velocities).finished());

  // Set the parameters
  std::vector<double> p = x0;
  p.insert(p.end(), x_desired_.begin(), x_desired_.end());
  p.insert(p.end(), u_prev_.begin(), u_prev_.end());
  args_["p"] = p;

  // Solve NLP. If a warm-started solve fails, e.g. because the state jumped away from the
  // previous solution, retry once from a cold start at the current state.
  casadi::DMDict sol;
  std::string error;
  for (int attempt = 0; attempt < 2; ++attempt) {
    // Initial guess: the previous solution, or the initial state held over the horizon
    const bool warm = warm_start_;
    if (!warm) {
      auto guess = repeat(x0, N + 1);
      guess.resize(guess.size() + nu_ * N, 0.0);
      args_["x0"] = guess;
      // No multipliers from a previous solution (they start at zero).
      args_.erase("lam_x0");
      args_.erase("lam_g0");
    }
    sol = solver_(args_);
    error.clear();

    // A single SQP iteration ends with Maximum_Iterations_Exceeded by design.
    const auto stats = solver_.stats();
    const auto status = stats.at("return_status").as_string();
    if (!stats.at("success").as_bool() &&
        !(options_.solver == "sqpmethod" && status == "Maximum_Iterations_Exceeded")) {
      error = "The MPC solver did not converge: " + status;
    }
    // A failed QP within an SQP iteration is not reported, so check that the solution starts
    // at the current state.
    const std::vector<double> start = sol.at("x").nonzeros();
    for (casadi_int i = 0; error.empty() && i < nx_; ++i) {
      if (!(std::abs(start[i] - x0[i]) < 1e-3)) {
        error = "The MPC solver failed: the solution does not start at the current state.";
      }
    }
    warm_start_ = false;
    if (error.empty() || !warm) {
      break;
    }
  }
  if (!error.empty()) {
    return tl::make_unexpected(error);
  }
  const std::vector<double> opt = sol.at("x").nonzeros();
  // Warm start the next solve from this solution: x0 is the initial guess of [X; U], and
  // lam_x0 / lam_g0 (CasADi's names) are the Lagrange multipliers of the bounds (lbx, ubx) and
  // of the constraints g (dynamics, torques), i.e. which limits are active.
  args_["x0"] = sol.at("x");
  args_["lam_x0"] = sol.at("lam_x");
  args_["lam_g0"] = sol.at("lam_g");
  warm_start_ = true;

  // Optimal states and inputs
  const auto x_opt = [&](casadi_int j) {
    return Eigen::Map<const Eigen::VectorXd>(&opt[j * nx_], nx_);
  };
  const auto u_opt = [&](casadi_int j) {
    return Eigen::Map<const Eigen::VectorXd>(&opt[nx_ * (N + 1) + j * nu_], nu_);
  };
  u_prev_ = toStd(u_opt(0));

  // Predicted trajectory x_0 ... x_N
  JointTrajectory trajectory;
  trajectory.joint_names = joint_group_info_.joint_names;
  for (casadi_int k = 0; k <= N; ++k) {
    trajectory.times.push_back(static_cast<double>(k) * Ts_);
    trajectory.positions.push_back(x_opt(k).head(nu_));
    trajectory.velocities.push_back(x_opt(k).tail(nu_));
  }
  return trajectory;
}

}  // namespace roboplan
