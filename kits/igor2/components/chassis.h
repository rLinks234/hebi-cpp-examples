#pragma once

#include <chrono>
#include <memory>

#include "components/body.h"

namespace hebi {

class Chassis : public BaseBody {

private:

  static constexpr NumCommands = 6;

  void create_trajectory();

  PIDController velocity_controller_(15.0, 0.1, 0.3);
  double user_commanded_directional_velocity_{0.0};
  double user_commanded_yaw_velocity_{0.0};
  double minimum_ramp_time_{0.5};
  double hip_pitch_{0.0};
  double hip_pitch_velocity_{0.0};

  //----------------------------------------------------------------------------
  // Velocity controller fields
  double lean_feedforward_{0.0};
  double velocity_feedforward_{0.0};
  double velocity_error_{0.0};
  double velocity_error_cumulative_{0.0};
  double lean_angle_error_{0.0};
  double lean_angle_error_cumulative_{0.0};
  double last_command_chassis_velocity_{0.0};
  double last_feedback_chassis_velocity_{0.0};
  
  double calculated_lean_angle_{0.0};

  //----------------------------------------------------------------------------
  // Trajectory fields
  Eigen::Matrix<double, NumCommands, 2> velocities_;
  Eigen::Matrix<double, NumCommands, 2> accelerations_;
  Eigen::Matrix<double, NumCommands, 2> jerks_;
  std::shared_ptr<trajectory::Trajectory> trajectory_;
  std::chrono::high_resolution_clock::time_point trajectory_time_;

public:

  struct VelocityControllerParams {
    double left_wheel_velocity{0.0};
    double right_wheel_velocity{0.0};
    double wheel_radius{0.0};
    double height_com{0.0};
    double robot_mass{0.0};
    double feedback_lean_angle{0.0};
    double feedback_lean_angle_velocity{0.0};
  };

  Chassis() = delete;
  Chassis(const Chassis&) = delete;
  Chassis(Chassis&&) = delete;
  virtual ~Chassis() = default;

  Chassis(std::mutex& lock, double mass=6.0,
          double com_x=0.0, double com_y=0.0, double com_z=0.13);

  /**
   * 
   */
  void update_time();

  /**
   * 
   */
  void update_trajectory(double user_commanded_knee_velocity,
                         const Vector3d& user_commanded_group_velocity);

  /**
   * 
   */
  void integrate_step(double dt);

  /**
   * 
   */
  void update_velocity_controller(double dt, const VelocityControllerParams& params);

//------------------------------------------------------------------------------
// Mutators

  void set_directional_velocity(double velocity);
  void set_yaw_velocity(double velocity);

//------------------------------------------------------------------------------
// Accessors

  double calculated_knee_velocity() const {
    return velocities_(2, 0);
  }

  double calculated_yaw_velocity() const {
    return velocities_(1, 0);
  }

  double velocity_feedforward() const {
    return velocity_feedforward_;
  }

  double lean_angle_error() const {
    return lean_angle_error_;
  }

  double lean_angle_error_cumulative() const {
    return lean_angle_error_cumulative_;
  }

  Vector3d calculated_grip_velocity() const {
    Vector3d ret;
    ret = velocities_.segment<3>(3);
    return ret;
  }



};

}
