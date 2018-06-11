#include "components/body.h"
#include "components/arm.h"
#include "components/leg.h"

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
  current_jacobians_actual_ = jacobian_endeffector.topRightCorner<6, DoFCount>();

  pos.segment<DoFCount>(0) = feedback_position_command_;
  robot_.getJEndEffector(pos, jacobian_endeffector);
  current_jacobians_expected_ = jacobian_endeffector.topRightCorner<6, DoFCount>();

  for (size_t i = 0; i < CoMFrameCount; i++) {
    Eigen::Matrix4d& mat = current_coms_[i];
    current_xyz_.col(i) = mat.topRightCorner<3, 1>();
  }

  {
    const double inv_mass = 1.0 / mass();
    Eigen::Matrix<double, CoMFrameCount, 3> com_calc_m;
    com_calc_m.col(0) = masses_;
    com_calc_m.col(1) = masses_;
    com_calc_m.col(2) = masses_;

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
  Eigen::VectorXd current_positions = Eigen::VectorXd(DoFCount);

  MatrixXd positions(DoFCount, 2);
  MatrixXd zeros = MatrixXd::Zero(DoFCount, 2);
  VectorXd time(2);

  time[0] = 0.0;
  time[1] = duration;

  for (size_t i = 0; i < DoFCount; i++) {
    positions(i, 0) = position[group_indices_[i]];
    positions(i, 1) = home_angles_[i];
  }

  return trajectory::Trajectory::createUnconstrainedQp(time, positions,
                                                       &zeros, &zeros);
}

template <size_t DoFCount, size_t OutputFrameCount, size_t CoMFrameCount>
void PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>::
get_grav_comp_efforts(const Eigen::VectorXd& positions,
                        const Vector3d& gravity,
                        Eigen::Matrix<double, DoFCount, 1>& comp_torque) {
  // Normalize gravity vector (to 1g, or 9.8 m/s^2)
  Eigen::Vector3d normed_gravity = gravity;
  if (normed_gravity.norm() > 0.0) {
    normed_gravity /= normed_gravity.norm();
    normed_gravity *= 9.81;
  }

  hebi::robot_model::MatrixXdVector jacobians;
  robot_.getJ(HebiFrameTypeCenterOfMass, positions, jacobians);

  // Get torque for each module
  // comp_torque = J' * wrench_vector
  // (for each frame, sum this quantity)
  comp_torque.setZero();

  // Wrench vector
  Eigen::Matrix<double, 6, 1> wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
  wrench_vec.setZero();
  for (size_t i = 0; i < CoMFrameCount; ++i) {
    // Set translational part
    for (size_t j = 0; j < 3; ++j) {
      wrench_vec[j] = -normed_gravity[j] * masses_[i];
    }

    // Add the torques for each joint to support the mass at this frame
    comp_torque += (jacobians[i].transpose() * wrench_vec).segment<DoFCount>(0);
  }
}

template void LegBase::update_position();
template void ArmBase::update_position();
template std::shared_ptr<hebi::trajectory::Trajectory> LegBase::create_home_trajectory(const Eigen::VectorXd& position, double duration);
template std::shared_ptr<hebi::trajectory::Trajectory> ArmBase::create_home_trajectory(const Eigen::VectorXd& position, double duration);
template void LegBase::get_grav_comp_efforts(const Eigen::VectorXd& positions, const Vector3d& gravity, Eigen::Matrix<double, 2, 1>& comp_torque);
template void ArmBase::get_grav_comp_efforts(const Eigen::VectorXd& positions, const Vector3d& gravity, Eigen::Matrix<double, 4, 1>& comp_torque);

}
