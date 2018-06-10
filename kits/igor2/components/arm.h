#pragma once

#include "components/body.h"
#include "group_command.hpp"

namespace hebi {

using ArmBase = PeripheralBody<4, 7, 7>;

template <bool NegateDirection>
class Arm : public ArmBase {

public:

  static constexpr size_t NumOfDofs = 4;
  static constexpr size_t OutputFrameCount = 7;
  static constexpr size_t CoMFrameCount = 7;

private:

  static constexpr double Direction = DirectionResolver<NegateDirection>::Direction;

  void setup_arm();

  template<typename S> using VectorDoF = ArmBase::VectorDoF<S>;
  template<typename S> using JacobianMatrix = ArmBase::JacobianMatrix<S>;

  double jacobian_determinant_threshold_;
  double current_determinant_actual_;
  double current_determinant_expected_;
  double user_commanded_wrist_velocity_;

  alignas(16) Eigen::Vector3d user_commanded_grip_velocity_;
  alignas(16) Eigen::Vector3d grip_position_;
  alignas(16) Eigen::Vector3d new_grip_position_;

  alignas(16) VectorDoF<double> joint_angles_;
  alignas(16) VectorDoF<double> grav_comp_torque_;
  alignas(16) VectorDoF<double> joint_velocities_;
  alignas(16) VectorDoF<double> joint_efforts_;

  alignas(64) Eigen::Array<double, 6, 1> damper_gains_;
  alignas(64) Eigen::Array<double, 6, 1> spring_gains_;

  alignas(64) Eigen::Matrix4d home_ef_;

protected:

  void update_position() override;

public:

  Arm() = delete;
  Arm(const Arm&) = delete;
  Arm(Arm&&) = delete;
  virtual ~Arm() = default;

  Arm(std::mutex& lock, std::array<size_t, 4> group_indices)
    : ArmBase(lock, std::move(group_indices)) {
    setup_arm();
  }

  /**
   * 
   */
  void integrate_step(double dt, const Eigen::Vector3d& calculated_grip_velocity);

  /**
   * 
   */
  void update_command(hebi::GroupCommand& group_command,
                      const Eigen::Matrix4d& pose,
                      double soft_start);

  void set_x_velocity(double velocity);
  void set_y_velocity(double velocity);
  void set_z_velocity(double velocity);
  void set_wrist_velocity(double velocity);

  const Eigen::Vector3d& user_commanded_grip_velocity() const {
    return user_commanded_grip_velocity_;
  }

  double user_commanded_wrist_velocity() const {
    return user_commanded_wrist_velocity_;
  }

};

}
