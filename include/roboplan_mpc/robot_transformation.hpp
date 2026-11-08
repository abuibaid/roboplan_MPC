#pragma once

#include <map>
#include <memory>
#include <string>

#include <casadi/casadi.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <roboplan/core/scene.hpp>

namespace roboplan {

/// @brief Forward kinematics of a joint group on CasADi symbols, from the Pinocchio model.
/// @details The homogeneous transformations T_0_i from the world frame to each joint of the group
/// and to the tip frame, as symbolic expressions of the joint positions, for use in the
/// constraints of an optimal control problem. Replaces a hand-written chain of DH matrices: the
/// chain is taken from the robot model (URDF) in the scene, so it works for any robot.
class RobotTransformation {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene of the robot.
  /// @param group_name The joint group whose joint positions are the symbols.
  /// @param tip_frame The tool frame, e.g. "tool0", or the TCP frame of a gripper in the URDF.
  RobotTransformation(const std::shared_ptr<Scene> scene, const std::string& group_name,
                      const std::string& tip_frame);

  /// @brief Computes the transformations for the joint positions q of the group.
  /// @param q The joint positions of the group (symbols or numbers), size nq of the group.
  /// @return The 4x4 transformations [R p; 0 1] in the world frame:
  ///   T["T1"] ... T["Tn"]: the n joints of the group, in order,
  ///   T["T(n+1)"]:         the tip frame, e.g. T["T7"] for the UR5,
  ///   T["<frame name>"]:   every frame of the model, by its URDF name.
  ///   Joints outside the group are held at their current positions in the scene.
  std::map<std::string, casadi::SX> robotTransformation(const casadi::SX& q) const;

private:
  /// @brief The Pinocchio model on CasADi symbols.
  pinocchio::ModelTpl<casadi::SX> ad_model_;

  /// @brief The joint group info.
  JointGroupInfo joint_group_info_;

  /// @brief The tip frame name.
  std::string tip_frame_;

  /// @brief The positions of all joints, for the joints outside the group.
  Eigen::VectorXd q_current_;
};

}  // namespace roboplan
