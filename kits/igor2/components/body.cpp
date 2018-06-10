#include "components/body.h"

namespace hebi {

template <size_t DoFCount, size_t OutputFrameCount, size_t CoMFrameCount>
void PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>::
  update_position() {

  static_assert(OutputFrameCount > 0, "OutputFrameCount must be > 0");
  static_assert(DoFCount > 0, "DoFCount must be > 0");
  static_assert(CoMFrameCount > 0, "CoMFrameCount must be > 0");

  robot_model::Matrix4dVector com_frames(CoMFrameCount);
  robot_model::Matrix4dVector fk_frames(OutputFrameCount);
  Eigen::VectorXd pos(DoFCount);
  pos.segment<DoFCount>(0) = feedback_position_;

  robot_.getFK(HebiFrameTypeCenterOfMass, pos, com_frames);
  robot_.getFK(HebiFrameTypeOutput, pos, fk_frames);

  for (size_t i = 0; i < OutputFrameCount; i++) {
    current_coms_[i] = com_frames[i];
  }

  for (size_t i = 0; i < CoMFrameCount; i++) {
    current_fk_[i] = fk_frames[i];
  }

  current_tip_fk_ = current_fk_[OutputFrameCount - 1];

  Eigen::MatrixXd jacobian_endeffector(6, DoFCount);
  robot_.getJEndEffector(pos, jacobian_endeffector);
  // TODO: Don't do this so hackily
  std::memcpy(current_jacobians_actual_.data(), jacobian_endeffector.data(), sizeof(6 * DoFCount * sizeof(double)));

  pos.segment<DoFCount>(0) = feedback_position_command_;
  robot_.getJEndEffector(pos, jacobian_endeffector);
  // TODO: Don't do this so hackily
  std::memcpy(current_jacobians_expected_.data(), jacobian_endeffector.data(), sizeof(6 * DoFCount * sizeof(double)));

  for (size_t i = 0; i < CoMFrameCount; i++) {
    Eigen::Matrix4d& mat = current_coms_[i];
    current_xyz_.col(i) = mat.topRightCorner<3, 1>();
  }

  {
    const double inv_mass = 1.0 / mass();
    Eigen::Matrix<double, CoMFrameCount, 3> com_calc_m;
    com_calc_m.col(0) = masses_; // masses_.transpose()
    com_calc_m.col(1) = masses_; // masses_.transpose()
    com_calc_m.col(2) = masses_; // masses_.transpose()

    const Eigen::Matrix3d prod = current_xyz_ * com_calc_m;
    const double com_x = prod.row(0).sum() * inv_mass;
    const double com_y = prod.row(1).sum() * inv_mass;
    const double com_z = prod.row(2).sum() * inv_mass;

    set_com(com_x, com_y, com_z);
  }

}

template <size_t DoFCount, size_t OutputFrameCount, size_t CoMFrameCount>
std::shared_ptr<hebi::trajectory::Trajectory> PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>::
  create_home_trajectory(const Eigen::VectorXd& position,
                         double duration) {
  // TODO
}

template <size_t DoFCount, size_t OutputFrameCount, size_t CoMFrameCount>
VectorXd PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>::
  get_grav_comp_efforts(const Eigen::VectorXd& positions,
                        const Vector3d& gravty) {
  // TODO
}

}
