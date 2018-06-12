#include "components/arm.h"
#include "util/affine3d.h"
#include "util/grav_comp.hpp"

#if !defined(__has_builtin)
#define __has_builtin(x) 0
#endif

#if __has_builtin(cxx_if_constexpr)
#define hebi_if_constexpr(X) if constexpr(X)
#else
#define hebi_if_constexpr(X) if(X)
#endif

namespace hebi {

template <size_t NumDoFs, size_t CoMFrames>
static void get_grav_comp(
  robot_model::RobotModel& model,
  const Eigen::Matrix<double, NumDoFs, 1>& positions,
  const Eigen::Matrix<double, CoMFrames, 1>& masses,
  const Eigen::Vector3d& gravity,
  Eigen::Matrix<double, NumDoFs, 1>& comp_torque) {
    // Normalize gravity vector (to 1g, or 9.8 m/s^2)
    Eigen::Vector3d normed_gravity = gravity;
    if (normed_gravity.norm() > 0.0) {
      normed_gravity /= normed_gravity.norm();
      normed_gravity *= 9.81;
    }

    hebi::robot_model::MatrixXdVector jacobians;
    model.getJ(HebiFrameTypeCenterOfMass, positions, jacobians);

    // Get torque for each module
    // comp_torque = J' * wrench_vector
    // (for each frame, sum this quantity)
    comp_torque.setZero();

    // Wrench vector
    Eigen::Matrix<double, 6, 1> wrench_vec(6); // For a single frame; this is (Fx/y/z, tau x/y/z)
    wrench_vec.setZero();
    for (size_t i = 0; i < CoMFrames; ++i) {
      // Set translational part
      for (size_t j = 0; j < 3; ++j) {
        wrench_vec[j] = -normed_gravity[j] * masses[i];
      }

      // Add the torques for each joint to support the mass at this frame
      comp_torque += jacobians[i].transpose() * wrench_vec;
    }
}

//------------------------------------------------------------------------------
// Private Functions

template <bool NegateDirection>
void Arm<NegateDirection>::setup_arm() {
  damper_gains_  << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  spring_gains_  << 100.0, 10.0, 100.0, 0.0, 0.0, 0.0;

  Matrix4d base_frame = Matrix4d::Identity();
  base_frame(2, 3) = 0.20;
  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_4);
  robot_model::RobotModel::BracketType mounting;

  constexpr double shoulder_home_angle = 0.3490658503988659; // 20 degrees
  constexpr double elbow_home_angle = 1.0471975511965976;    // 60 degrees

  home_angles_[0] = 0.0;
  home_angles_[3] = 0.0;

  hebi_if_constexpr(NegateDirection) {
  // Right arm
    mounting = robot_model::RobotModel::BracketType::X5HeavyRightInside;
  } else {
  // Left Arm
    mounting = robot_model::RobotModel::BracketType::X5HeavyLeftInside;
  }

  base_frame(1, 3) = Direction*0.10;
  home_angles_[1] = Direction*shoulder_home_angle;
  home_angles_[2] = Direction*elbow_home_angle;

  robot_.addBracket(mounting);
  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_9);
  robot_.addLink(robot_model::RobotModel::LinkType::X5, 0.325, 0.0);
  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_4);
  robot_.addLink(robot_model::RobotModel::LinkType::X5, 0.325, M_PI);
  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_4);

  robot_.setBaseFrame(base_frame);

  robot_.getEndEffector(home_angles_, home_ef_);

  Eigen::VectorXd masses(CoMFrameCount);
  robot_.getMasses(masses);
  set_mass(masses.sum());
  masses_ = masses.segment<CoMFrameCount>(0);

  user_commanded_grip_velocity_.setZero();
  grip_position_ = home_ef_.topRightCorner<3, 1>();
  new_grip_position_ = grip_position_;
  joint_angles_ = home_angles_;
  joint_velocities_.setZero();
  joint_efforts_.setZero();
  grav_comp_torque_.setZero();

}

//------------------------------------------------------------------------------
// Protected Functions

template <bool NegateDirection>
void Arm<NegateDirection>::update_position() {
  ArmBase::update_position();
  current_determinant_actual_ = current_jacobians_actual_.topLeftCorner<3, 3>().determinant();
  current_determinant_expected_ = current_jacobians_expected_.topLeftCorner<3, 3>().determinant();
}

//------------------------------------------------------------------------------
// Public Functions

template <bool NegateDirection>
void Arm<NegateDirection>::integrate_step(double dt,
                                          const Vector3d& calculated_grip_velocity) {
  Vector3d adjusted_grip_velocity = calculated_grip_velocity;
  hebi_if_constexpr(NegateDirection) {
    adjusted_grip_velocity[1] = -adjusted_grip_velocity[1];
  }

  adjusted_grip_velocity *= dt;
  new_grip_position_ = adjusted_grip_velocity + grip_position_;

  auto xyz_objective = robot_model::EndEffectorPositionObjective(new_grip_position_);
  // FIXME: Get rid of these temporaries
  VectorXd new_arm_joint_angles(NumOfDofs);
  VectorXd initial_pos(NumOfDofs);
  MatrixXd jacob(6, NumOfDofs);
  initial_pos.segment<NumOfDofs>(0) = feedback_position_.segment<NumOfDofs>(0); // initial_pos[0:NumDofs] = feedback_position[0:NumDofs]
  auto res = robot_.solveIK(initial_pos, new_arm_joint_angles, xyz_objective);

  robot_.getJacobianEndEffector(new_arm_joint_angles, jacob);
  double determinant_jacobian_new = jacob.topLeftCorner<3, 3>().determinant();

  if ((current_determinant_expected_ < jacobian_determinant_threshold_) &&
      (determinant_jacobian_new < current_determinant_expected_)) {
    // Near singularity - don't command arm towards it
    joint_velocities_.setZero();
  } else {
    // TODO: Check w/ matt and dave if solving w/ LU factorization makes sense here
    joint_velocities_.segment<3>(0) = current_jacobians_actual_.topLeftCorner<3, 3>().
        fullPivLu().solve(user_commanded_grip_velocity_);
    joint_angles_.segment<3>(0) = new_arm_joint_angles.segment<3>(0);
    grip_position_ = new_grip_position_;
  }

  double wrist_velocity = user_commanded_wrist_velocity_;
  hebi_if_constexpr(NegateDirection) {
    wrist_velocity = -wrist_velocity;
  }

  joint_velocities_[3] = joint_velocities_[1] + joint_velocities_[2] + wrist_velocity;
  joint_angles_[3] = joint_angles_[3] + (joint_velocities_[3] * dt);
}

template <bool NegateDirection>
void Arm<NegateDirection>::update_command(hebi::GroupCommand& group_command,
                                          const Matrix4d& pose,
                                          double soft_start) {
  xyz_error_ = grip_position_-current_tip_fk_.topRightCorner<3, 1>();
  position_error_.segment<3>(0) = xyz_error_;
  position_error_.array() *= spring_gains_;
  velocity_error_ = current_jacobians_actual_ * (joint_velocities_ - feedback_velocity_);
  velocity_error_.array() *= damper_gains_;
  impedance_error_ = position_error_ + velocity_error_;
  impedance_torque_ = current_jacobians_actual_.transpose() * impedance_error_;
  Eigen::Vector3d gravity = -pose.topRightCorner<3, 1>();
  get_grav_comp<4, 7>(robot_, feedback_position_, masses_, gravity, grav_comp_torque_);
  joint_efforts_ = (impedance_torque_ * soft_start) + grav_comp_torque_;

  size_t idx = 0;
  for (auto i : group_indices()) {
    auto& actuator = group_command[i].actuator();
    actuator.position().set(joint_angles_[idx]);
    actuator.velocity().set(static_cast<float>(joint_velocities_[idx]));
    actuator.effort().set(static_cast<float>(joint_efforts_[idx]));
    idx++;
  }
}

template <bool NegateDirection>
void Arm<NegateDirection>::set_x_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_grip_velocity_[0] = velocity; 
}

template <bool NegateDirection>
void Arm<NegateDirection>::set_y_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_grip_velocity_[1] = velocity;
}

template <bool NegateDirection>
void Arm<NegateDirection>::set_z_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_grip_velocity_[2] = velocity;
}

template <bool NegateDirection>
void Arm<NegateDirection>::set_wrist_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_wrist_velocity_ = velocity;
}

template<bool B> constexpr size_t Arm<B>::NumOfDofs;
template<bool B> constexpr size_t Arm<B>::OutputFrameCount;
template<bool B> constexpr size_t Arm<B>::CoMFrameCount;

template void Arm<false>::setup_arm();
template void Arm<true>::setup_arm();
template void Arm<false>::update_position();
template void Arm<true>::update_position();
template void Arm<false>::integrate_step(double dt, const Vector3d& calculated_grip_velocity);
template void Arm<true>::integrate_step(double dt, const Vector3d& calculated_grip_velocity);
template void Arm<false>::update_command(hebi::GroupCommand& group_command, const Matrix4d& pose, double soft_start);
template void Arm<true>::update_command(hebi::GroupCommand& group_command, const Matrix4d& pose, double soft_start);
template void Arm<false>::set_x_velocity(double velocity);
template void Arm<true>::set_x_velocity(double velocity);
template void Arm<false>::set_y_velocity(double velocity);
template void Arm<true>::set_y_velocity(double velocity);
template void Arm<false>::set_z_velocity(double velocity);
template void Arm<true>::set_z_velocity(double velocity);
template void Arm<false>::set_wrist_velocity(double velocity);
template void Arm<true>::set_wrist_velocity(double velocity);

}