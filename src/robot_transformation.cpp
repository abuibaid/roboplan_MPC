#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <roboplan_mpc/robot_transformation.hpp>

namespace roboplan {

using casadi::SX;

RobotTransformation::RobotTransformation(const std::shared_ptr<Scene> scene,
                                         const std::string& group_name,
                                         const std::string& tip_frame)
    : tip_frame_{tip_frame} {
  joint_group_info_ = scene->getJointGroupInfo(group_name).value();
  ad_model_ = scene->getModel().cast<SX>();
  q_current_ = scene->getCurrentJointPositions();
}

std::map<std::string, SX> RobotTransformation::robotTransformation(const SX& q) const {
  const auto n = static_cast<casadi_int>(joint_group_info_.q_indices.size());

  // ###### Joint positions ######
  // q_full = [q of the group; current positions of the other joints]
  Eigen::Matrix<SX, Eigen::Dynamic, 1> q_full(ad_model_.nq);
  for (Eigen::Index i = 0; i < ad_model_.nq; ++i) {
    q_full[i] = q_current_[i];
  }
  for (casadi_int i = 0; i < n; ++i) {
    q_full[joint_group_info_.q_indices[i]] = q(i);
  }

  // ###### Forward kinematics (Pinocchio) ######
  // T_0_i = T_0_1 T_1_2 ... T_(i-1)_i, each T from the joint placements of the URDF (instead of
  // DH parameters) and the joint position, for every frame of the model.
  pinocchio::DataTpl<SX> ad_data(ad_model_);
  pinocchio::framesForwardKinematics(ad_model_, ad_data, q_full);

  // T = [R p; 0 1]
  const auto homogeneous = [](const pinocchio::SE3Tpl<SX>& placement) {
    SX T = SX::eye(4);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        T(r, c) = placement.rotation()(r, c);
      }
      T(r, 3) = placement.translation()(r);
    }
    return T;
  };

  std::map<std::string, SX> T;
  for (size_t f = 0; f < ad_model_.frames.size(); ++f) {
    T[ad_model_.frames[f].name] = homogeneous(ad_data.oMf[f]);
  }
  // The joints of the group in order, T1 ... Tn, and the tip as T(n+1)
  for (size_t j = 0; j < joint_group_info_.joint_names.size(); ++j) {
    T["T" + std::to_string(j + 1)] = T.at(joint_group_info_.joint_names[j]);
  }
  T["T" + std::to_string(n + 1)] = T.at(tip_frame_);
  return T;
}

}  // namespace roboplan
