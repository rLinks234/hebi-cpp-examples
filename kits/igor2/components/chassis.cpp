#include "components/chassis.h"

namespace hebi {

Chassis::Chassis(std::mutex& lock, double mass, double com_x,
                 double com_y, double com_z)
  : BaseBody(lock, mass, com_x, com_y, com_z) {

  velocities_.setZero();
  accelerations_.setZero();
  jerks_.setZero();

  create_trajectory();
}

//------------------------------------------------------------------------------
// Private Functions

void Chassis::create_trajectory() {
  MatrixXd velocities(NumCommands, 2);
  MatrixXd accelerations(NumCommands, 2);
  MatrixXd jerks(NumCommands, 2);
  VectorXd time(2);

  time[0] = 0.0;
  time[1] = minimum_ramp_time_;

  velocities.block<NumCommands, 2>(0, 0) = velocities_;
  accelerations.block<NumCommands, 2>(0, 0) = accelerations_;
  jerks.block<NumCommands, 2>(0, 0) = jerks_;

  trajectory_ = trajectory::Trajectory::createUnconstrainedQp(time, velocities,
                                                              &accelerations,
                                                              &jerks);
}

//------------------------------------------------------------------------------
// Public Functions

void Chassis::update_time() {
  trajectory_time_ = std::chrono::high_resolution_clock::now();
}

void Chassis::update_trajectory(double user_commanded_knee_velocity,
                                const Vector3d& user_commanded_grip_velocity) {
  auto time_now = std::chrono::high_resolution_clock::now();
  auto duration = time_now - trajectory_time_;
  trajectory_time_ = time_now;
  
  const double t = std::chrono::duration_cast<std::chrono::seconds>(duration);

  // ---------------------------------------------
  // Smooth the trajectories for various commands.
  // This will be the starting waypoint for the new trajectory.
  // The end waypoint will be the desired (user commanded) values.
  VectorXd velocity_now(NumCommands);
  VectorXd acceleration_now(NumCommands);
  VectorXd jerk_now(NumCommands);
  trajectory_->getState(t, &velocity_now, &acceleration_now, &jerk_now);

  // Start waypoint
  velocities_.col(0) = velocity_now;
  acclerations_.col(0) = acceleration_now;
  jerks_.col(0) = jerk_now;

  // End waypoint
  velocities_(0, 1) = user_commanded_directional_velocity_;
  velocities_(1, 1) = user_commanded_yaw_velocity_;
  velocities_(2, 1) = user_commanded_knee_velocity;
  velocities_.bottomRightCorner<3, 1>() = user_commanded_grip_velocity;

  create_trajectory();
}

void Chassis::integrate_step(double dt) {
  hip_pitch_ += hip_pitch_velocity_ * dt;
}

static inline bool both_finite(double a, double b) {
  if (!std::isfinite(a)) {
    return false;
  }
  return std::isfinite(b);
}

static inline double clip(double val, double low, double high) {
  if (val > high) {
    return high;
  } else if (val < low) {
    return low;
  }
  return val;
}

void Chassis::update_velocity_controller(double dt,
                                         const VelocityControllerParams& params) {

  // Consts are liberally pasted everywhere to encourage the compiler
  // to take advantage of constant propagation. A C++ compliant compiler
  // could refuse to propagate non constant local variables.
  const double velocity_P = velocity_controller_.P();
  const double velocity_I = velocity_controller_.I();
  const double velocity_D = velocity_controller_.D();
  const double inv_dt = 1.0 / dt;
  const double wheel_radius = params.wheel_radius;
  const double l_wheel_vel = params.left_wheel_velocity;
  const double r_wheel_vel = params.right_wheel_velocity;
  
  double fbk_chassis_vel = params.height_com * params.feedback_lean_angle_velocity;

  if (both_finite(l_wheel_vel, r_wheel_vel)) {
    fbk_chassis_vel += wheel_radius * (l_wheel_vel - r_wheel_vel) * 0.5;
  }

  const double cmd_chassis_vel = velocities_(0, 0);
  const double chassis_vel_error = cmd_chassis_vel - fbk_chassis_vel;
  const double chassis_vel_error_cumulative = 
    clip(velocity_error_cumulative_ + (chassis_vel_error * dt), -50.0, 50.0);
  const double cmd_chassis_accel =
    (cmd_chassis_vel - last_command_chassis_velocity_) * inv_dt;
  const double fbk_chassis_accel =
    (fbk_chassis_vel - last_feedback_chassis_velocity_) * inv_dt;

  const double lean_feedforward = (0.1 * params.robot_mass) *
    cmd_chassis_accel / height_com;
  const double velocity_feedforward = cmd_chassis_vel / wheel_radius;

  const double cmd_lean_angle = (velocity_P * chassis_vel_error) +
    (velocity_I * chassis_vel_error_cumulative) +
    (velocity_D * fbk_chassis_accel) + lean_feedforward;

  const double lean_angle_error = params.feedback_lean_angle - cmd_lean_angle;
  const double lean_angle_error_cumulative =
    clip(lean_angle_error_cumulative_ + (lean_angle_error * dt), -0.2, 0.2);

  // Write back into global state
  lean_feedforward_ = lean_feedforward;
  velocity_feedforward_ = velocity_feedforward;
  velocity_error_ = chassis_vel_error;
  velocity_error_cumulative_ = chassis_vel_error_cumulative;
  lean_angle_error_ = lean_angle_error;
  lean_angle_error_cumulative_ = lean_angle_error_cumulative;
  last_command_chassis_velocity_ = cmd_chassis_vel;
  last_feedback_chassis_velocity_ = fbk_chassis_vel;
  calculated_lean_angle_ = cmd_lean_angle;
}

void Chassis::set_directional_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_directional_velocity_ = velocity;
}

void Chassis::set_yaw_velocity(double velocity) {
  auto lock = lock_guard();
  user_commanded_yaw_velocity_ = velocity;
}

}
