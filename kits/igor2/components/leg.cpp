#include "components/leg.h"
#include "util/rotation.h"

namespace hebi {

//------------------------------------------------------------------------------
// Private Functions

template<bool NegateDirection>
void Leg<NegateDirection>::setup_leg() {
  spring_gains_ << 2.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  damper_gains_ << 400.0, 0.0, 100.0, 0.0, 0.0, 0.0;
  roll_gains_ << 0.0, 0.0, 10.0, 0.0, 0.0, 0.0;

  const double pi_half = M_PI * 0.5;

  Matrix4d base_frame = Matrix4d::Identity();
  hip_transform_ = base_frame;

  util::rotateX<double, 4>(hip_transform_, pi_half);
  hip_transform_(1, 3) = 0.0225;
  hip_transform_(2, 3) = 0.055;

  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_9);
  robot_.addLink(robot_model::RobotModel::LinkType::X5, 0.375, M_PI);
  robot_.addActuator(robot_model::RobotModel::ActuatorType::X5_4);
  robot_.addLink(robot_model::RobotModel::LinkType::X5, 0.325, M_PI);

  home_knee_angle_ = 2.2689280275926285; // 130 degrees
  home_hip_angle_ = 2.705260340591211;   // 155 degrees

  base_frame(1, 3) = Direction * 0.15;
  util::rotateX<double, 4>(base_frame, -Direction * pi_half);
  home_angles_[0] = Direction * home_hip_angle_;
  home_angles_[1] = Direction * home_knee_angle_;

  robot_.setBaseFrame(base_frame);

  Eigen::VectorXd masses(4);
  robot_.getMasses(masses);
  set_mass(masses.sum());
  masses_ = masses.segment<4>(0);

}

template<bool NegateDirection>
bool Leg<NegateDirection>::should_limit_knee_velocity() {
  return ((knee_angle_ > knee_angle_max_) && (knee_velocity_ > 0.0) ||
    (knee_angle_ < knee_angle_min_) && (knee_velocity_ < 0.0));
}

//------------------------------------------------------------------------------
// Protected Functions

template<bool NegateDirection>
void Leg<NegateDirection>::update_position() {
  LegBase::update_position();
  robot_.getEndEffector(feedback_position_command_,
                        /*output=*/ current_command_tip_fk_);
}

//------------------------------------------------------------------------------
// Public Functions

template<bool NegateDirection>
void Leg<NegateDirection>::integrate_step(double dt, double knee_velocity) {
  if (should_limit_knee_velocity()) {
    knee_velocity = 0.0;
  }
  knee_velocity_ = knee_velocity;
  knee_angle_ += knee_velocity*dt;
  hip_angle_ = (M_PI+knee_angle_)*0.5;
}

template<bool NegateDirection>
void Leg<NegateDirection>::update_command(hebi::GroupCommand& group_command,
                                          double roll_angle,
                                          double soft_start) {
  const auto& indices = group_indices();
  const auto hip_index = indices[0];
  const auto knee_index = indices[1];
  auto& hip_command = group_command[hip_index].actuator();
  auto& knee_command = group_command[knee_index].actuator();
  // set angle to be in proper direction
  roll_angle *= Direction;

  // Calculate position and velocity
  const double knee_velocity = Direction*knee_velocity_;
  hip_command.position().set(Direction*hip_angle_);
  hip_command.velocity().set(static_cast<float>(knee_velocity*0.5));
  knee_command.position().set(Direction*knee_angle_);
  knee_command.velocity().set(static_cast<float>(knee_velocity));

  // ----------------
  // Calculate effort

  // Calculate the current positional error
  xyz_error_ = current_command_tip_fk_.topRightCorner<3, 1>() -
               current_tip_fk_.topRightCorner<3, 1>();
  position_error_.segment<3>(0) = xyz_error_;
  // Calculate the current velocity error by:
  //  multiplying the current jacobian at the endeffector frame
  // by the:
  //  difference of the commanded velocity feedfback and actual velocity feedback
  velocity_error_ = current_jacobians_actual_ * feedback_velocity_error_;

  // Piecewise multiply the error terms by the predefined gains
  for (size_t i = 0; i < 6; i++) {
    double position_error = position_error_[i];
    double velocity_error = velocity_error_[i];
    double roll_term = roll_gains_[i] * roll_angle;
    position_error *= spring_gains_[i];
    velocity_error *= damper_gains_[i];
    impedance_error_[i] = position_error + velocity_error + roll_term;
    position_error_[i] = position_error;
    velocity_error_[i] = velocity_error;
  }

  impedance_torque_ = current_jacobians_actual_.transpose() * impedance_error_;
  // prevent huge torques from being calculated, and avoid unnecessary
  // multiplies once startup phase is done
  if (soft_start < 1.0) {
    impedance_torque_ *= soft_start;
  }

  hip_command.effort().set(static_cast<float>(impedance_torque_[0]));
  knee_command.effort().set(static_cast<float>(impedance_torque_[1]));
}

template<bool NegateDirection>
void Leg<NegateDirection>::set_knee_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_knee_velocity_ = velocity;
}

template void Leg<false>::setup_leg();
template void Leg<true>::setup_leg();
template void Leg<false>::update_position();
template void Leg<true>::update_position();
template bool Leg<false>::should_limit_knee_velocity();
template bool Leg<true>::should_limit_knee_velocity();
template void Leg<false>::integrate_step(double dt, double knee_velocity);
template void Leg<true>::integrate_step(double dt, double knee_velocity);
template void Leg<false>::update_command(hebi::GroupCommand& group_command,
                                         double roll_angle,
                                         double soft_start);
template void Leg<true>::update_command(hebi::GroupCommand& group_command,
                                        double roll_angle,
                                        double soft_start);
template void Leg<false>::set_knee_velocity(double velocity);
template void Leg<true>::set_knee_velocity(double velocity);

}
