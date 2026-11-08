#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_mpc/robot_transformation.hpp>

namespace roboplan {

namespace {

std::shared_ptr<Scene> loadScene(const std::string& model_dir, const std::string& name) {
  const auto prefix = example_models::get_package_models_dir() / model_dir;
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  auto scene = std::make_shared<Scene>(
      "test_scene", loadUrdfSceneDescription(prefix / (name + ".urdf"), package_paths));
  if (const auto imported = scene->importSrdf(loadTextFile(prefix / (name + ".srdf")));
      !imported) {
    throw std::runtime_error(imported.error());
  }
  return scene;
}

// The symbolic transformations, evaluated at random configurations, match Pinocchio's numeric
// forward kinematics of the scene.
void expectMatchesForwardKinematics(const std::shared_ptr<Scene>& scene,
                                    const std::string& group_name, const std::string& tip_frame) {
  const auto group_info = scene->getJointGroupInfo(group_name).value();
  const auto n = static_cast<casadi_int>(group_info.q_indices.size());
  RobotTransformation kinematics(scene, group_name, tip_frame);

  const casadi::SX q = casadi::SX::sym("q", n);
  const auto T = kinematics.robotTransformation(q);
  std::vector<std::string> keys = {"T" + std::to_string(n + 1), tip_frame};
  std::vector<std::string> frames = {tip_frame, tip_frame};
  for (casadi_int j = 0; j < n; ++j) {
    keys.push_back("T" + std::to_string(j + 1));
    frames.push_back(group_info.joint_names[j]);
  }
  std::vector<casadi::SX> outputs;
  for (const auto& key : keys) {
    outputs.push_back(T.at(key));
  }
  const casadi::Function f("robot_transformation", {q}, outputs);

  scene->setRngSeed(0);
  for (int trial = 0; trial < 10; ++trial) {
    const Eigen::VectorXd q_full = scene->randomCollisionFreePositions().value();
    const Eigen::VectorXd q_group = q_full(group_info.q_indices);
    scene->setJointPositions(q_full);
    const auto values = f(std::vector<casadi::DM>{
        casadi::DM(std::vector<double>(q_group.data(), q_group.data() + n))});
    for (size_t k = 0; k < keys.size(); ++k) {
      const Eigen::Matrix4d expected = scene->forwardKinematics(q_full, frames[k]);
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          EXPECT_NEAR(static_cast<double>(values[k](r, c)), expected(r, c), 1e-9)
              << keys[k] << " (" << r << ", " << c << "), trial " << trial;
        }
      }
    }
  }
}

}  // namespace

TEST(RobotTransformationTest, MatchesForwardKinematicsUR5) {
  expectMatchesForwardKinematics(loadScene("ur_robot_model", "ur5_gripper"), "arm", "tool0");
}

TEST(RobotTransformationTest, MatchesForwardKinematicsSO101) {
  expectMatchesForwardKinematics(loadScene("so101_robot_model", "so101"), "arm",
                                 "gripper_frame_link");
}

}  // namespace roboplan
