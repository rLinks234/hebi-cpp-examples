#pragma once

#include "components/body.h"
#include "group_command.hpp"

namespace hebi {

using LegBase = PeripheralBody<2, 4, 4>;

template<bool NegateDirection>
class Leg : public PeripheralBody<2, 4, 4> {

private:

  static constexpr double Direction = DirectionResolver<NegateDirection>::Direction;

  void setup_leg();
  bool should_limit_knee_velocity();

  template<typename S> using VectorDoF = LegBase::VectorDoF<S>;
  template<typename S> using JacobianMatrix = LegBase::JacobianMatrix<S>;

  double home_knee_angle_;
  double home_hip_angle_;
  double knee_angle_min_{0.65};
  double knee_angle_max_{0.65};
  double hip_angle_{0.0};
  double knee_angle_{0.0};
  double knee_velocity_{0.0};
  double user_commanded_knee_velocity_{0.0};

  alignas(64) Vector6d spring_gains_;
  alignas(64) Vector6d damper_gains_;
  alignas(64) Vector6d roll_gains_;

  alignas(64) Eigen::Matrix4d current_command_tip_fk_;
  alignas(64) Eigen::Matrix4d hip_transform_;

protected:

  void update_position() override;

public:

  Leg() = delete;
  Leg(const Leg&) = delete;
  Leg(Leg&&) = delete;
  virtual ~Leg() = default;

  Leg(std::mutex& lock, std::array<size_t, 2> group_indices)
    : LegBase(lock, std::move(group_indices)) {
    setup_leg();
  }

  /**
   * 
   */
  void integrate_step(double dt, double knee_velocity);

  /**
   * 
   */
  void update_command(hebi::GroupCommand& group_command,
                      double roll_angle,
                      double soft_start);

//------------------------------------------------------------------------------
// Mutators

  /**
   * 
   */
  void set_knee_velocity(double velocity);

//------------------------------------------------------------------------------
// Accessors

  double user_commanded_knee_velocity() const {
    return user_commanded_knee_velocity_;
  }

  double knee_angle() const {
    return knee_angle_;
  }

  

};

}
