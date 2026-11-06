#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>

#include <pinocchio/algorithm/rnea.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_mpc/mpc.hpp>

namespace roboplan {

constexpr auto kGroupName = "arm";

MPCOptions groupOptions() {
  MPCOptions options;
  options.group_name = kGroupName;
  return options;
}

class RoboPlanMPCTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const auto yaml_path = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    const auto description = loadUrdfSceneDescription(urdf_path, package_paths);
    scene = std::make_shared<Scene>("test_scene", description);
    scene->importJointLimitsFromConfig(loadJointLimitsConfig(yaml_path));
    if (const auto imported = scene->importSrdf(loadTextFile(srdf_path)); !imported) {
      throw std::runtime_error(imported.error());
    }
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene;
};

TEST_F(RoboPlanMPCTest, ReachesGoalWithinLimits) {
  MPC mpc(scene, groupOptions());
  const auto [lower_q, upper_q] = scene->getPositionLimitVectors(kGroupName).value();
  const auto [min_dq, max_dq] = scene->getVelocityLimitVectors(kGroupName).value();
  const auto [min_ddq, max_ddq] = scene->getAccelerationLimitVectors(kGroupName).value();
  const auto& model = scene->getModel();
  pinocchio::Data data(model);

  JointConfiguration goal;
  goal.positions = Eigen::VectorXd{{0.8, -1.2, 1.3, -0.6, 1.2, 0.4}};
  mpc.setGoal(goal);

  JointConfiguration state;
  state.positions = Eigen::VectorXd{{0.0, -1.57, 1.57, 0.0, 1.57, 0.0}};
  state.velocities = Eigen::VectorXd::Zero(6);

  // Closed loop over 6 s. The robot follows the predicted trajectory, as a joint trajectory
  // controller would: it is at the first predicted state at the next control cycle.
  const int num_steps = static_cast<int>(6.0 / mpc.getTimeStep());
  for (int i = 0; i < num_steps; ++i) {
    const auto trajectory = mpc.solve(state);
    ASSERT_TRUE(trajectory.has_value()) << trajectory.error();
    ASSERT_EQ(trajectory->positions.size(), mpc.getHorizon() + 1);
    EXPECT_DOUBLE_EQ(trajectory->times[1], mpc.getTimeStep());

    // The applied acceleration and torque are within their limits.
    const Eigen::VectorXd ddq = (trajectory->velocities[1] - state.velocities) / mpc.getTimeStep();
    EXPECT_TRUE((ddq.array() >= min_ddq.array() - 1e-6).all());
    EXPECT_TRUE((ddq.array() <= max_ddq.array() + 1e-6).all());
    const Eigen::VectorXd tau =
        pinocchio::rnea(model, data, state.positions, state.velocities, ddq);
    EXPECT_TRUE((tau.cwiseAbs().array() <= model.effortLimit.array() + 1e-6).all());

    state.positions = trajectory->positions[1];
    state.velocities = trajectory->velocities[1];
    EXPECT_TRUE((state.positions.array() >= lower_q.array() - 1e-6).all());
    EXPECT_TRUE((state.positions.array() <= upper_q.array() + 1e-6).all());
    EXPECT_TRUE((state.velocities.array() >= min_dq.array() - 1e-6).all());
    EXPECT_TRUE((state.velocities.array() <= max_dq.array() + 1e-6).all());
  }

  EXPECT_LT((state.positions - goal.positions).cwiseAbs().maxCoeff(), 1e-3);
  EXPECT_LT(state.velocities.norm(), 1e-3);
}

TEST_F(RoboPlanMPCTest, ReachesGoalOnJointLimit) {
  // Regression test: the measured state can end slightly outside a joint limit (within the QP
  // tolerance), which must not make the next problem infeasible.
  MPC mpc(scene, groupOptions());
  const auto [lower_q, upper_q] = scene->getPositionLimitVectors(kGroupName).value();

  JointConfiguration goal;
  goal.positions = Eigen::VectorXd{{0.8, -1.2, upper_q[2], -0.6, 1.2, 0.4}};
  mpc.setGoal(goal);

  JointConfiguration state;
  state.positions = Eigen::VectorXd{{0.0, -1.57, 1.57, 0.0, 1.57, 0.0}};
  state.velocities = Eigen::VectorXd::Zero(6);
  const int num_steps = static_cast<int>(6.0 / mpc.getTimeStep());
  for (int i = 0; i < num_steps; ++i) {
    const auto trajectory = mpc.solve(state);
    ASSERT_TRUE(trajectory.has_value()) << trajectory.error();
    state.positions = trajectory->positions[1];
    state.velocities = trajectory->velocities[1];
    EXPECT_TRUE((state.positions.array() <= upper_q.array() + 1e-4).all());
  }
  EXPECT_LT((state.positions - goal.positions).cwiseAbs().maxCoeff(), 1e-3);
}

TEST_F(RoboPlanMPCTest, RecoversFromStateJump) {
  // Regression test: a state far from the previous solution (e.g. a new start after a previous
  // motion) can make the warm-started QP infeasible; the solve then restarts cold.
  MPC mpc(scene, groupOptions());
  scene->setRngSeed(0);
  const Eigen::VectorXi q_indices = scene->getJointGroupInfo(kGroupName).value().q_indices;
  for (int move = 0; move < 30; ++move) {
    JointConfiguration state;
    state.positions = scene->randomCollisionFreePositions().value()(q_indices);
    state.velocities = Eigen::VectorXd::Zero(6);
    JointConfiguration goal;
    goal.positions = scene->randomCollisionFreePositions().value()(q_indices);
    mpc.setGoal(goal);
    for (int i = 0; i < 10; ++i) {
      const auto trajectory = mpc.solve(state);
      ASSERT_TRUE(trajectory.has_value()) << "move " << move << ": " << trajectory.error();
      state.positions = trajectory->positions[1];
      state.velocities = trajectory->velocities[1];
    }
  }
}

TEST_F(RoboPlanMPCTest, UnknownSolver) {
  MPCOptions options = groupOptions();
  options.solver = "unknown";
  EXPECT_THROW(MPC(scene, options), std::runtime_error);
}

TEST_F(RoboPlanMPCTest, InvalidState) {
  MPC mpc(scene, groupOptions());
  JointConfiguration state;
  state.positions = Eigen::VectorXd::Zero(3);
  state.velocities = Eigen::VectorXd::Zero(3);
  EXPECT_FALSE(mpc.solve(state).has_value());

  JointConfiguration goal;
  goal.positions = Eigen::VectorXd::Zero(3);
  EXPECT_THROW(mpc.setGoal(goal), std::invalid_argument);
}

}  // namespace roboplan
