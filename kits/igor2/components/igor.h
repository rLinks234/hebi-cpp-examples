#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include <Eigen/Eigen>

#include "components/arm.h"
#include "components/chassis.h"
#include "components/leg.h"
#include "util/input/joystick.h"

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"

namespace hebi {

class Igor {

private:

  friend class IgorAccessor;

  static constexpr size_t NumImuModules = 6;
  static constexpr size_t NumDoFs = 14;

  // 
  std::mutex value_lock_;
  std::condition_variable finished_cv_;
  std::mutex state_lock_;

  //----------------------------------------------------------------------------
  // Parameter Fields
  PIDController lean_controller_{1.0, 20.0, 10.0};
  double joystick_dead_zone_{0.06};
  double wheel_radius_{0.100};
  double wheel_base_{0.43};
  double max_wheel_velocity_{10.0};
  std::string gains_file_{"./resources/igorGains.xml"};

  //----------------------------------------------------------------------------
  // General state fields
  bool started_{false};
  bool balance_controller_enabled_{true};
  bool quit_flag_{false};
  std::chrono::high_resolution_clock::time_point start_time_;
  std::chrono::high_resolution_clock::time_point stop_time_;
  size_t num_spins_{0};
  std::array<uint64_t, NumDoFs> last_rx_time_{0};
  std::array<uint64_t, NumDoFs> diff_rx_time_{0};

  //----------------------------------------------------------------------------
  // HEBI interface fields

  std::shared_ptr<Group> group_;
  GroupCommand group_command_{NumDoFs};
  GroupFeedback group_feedback_{NumDoFs};
  std::shared_ptr<util::Joystick> joystick_;

  //----------------------------------------------------------------------------
  // Bodies
  Chassis chassis_{value_lock_};
  Leg<false> left_leg_{value_lock_, {2, 3}};
  Leg<true> right_leg_{value_lock_, {4, 5}};
  Arm<false> left_arm_{value_lock_, {6, 7, 8, 9}};
  Arm<true> right_arm_{value_lock_, {10, 11, 12, 13}};

  //----------------------------------------------------------------------------

  std::array<size_t, NumImuModules> imu_modules_{0, 1, 2, 3, 4, 5};

  double mass_{0.0};
  double roll_angle_{0.0};
  double pitch_angle_{0.0};
  double feedback_lean_angle_{0.0};
  double feedback_lean_angle_velocity_{0.0};
  double height_com_{0.0};

  Eigen::Vector3d com_;
  Eigen::Vector3d line_com_;
  Eigen::Vector3d ground_point_;

  Eigen::Matrix<double, 1, 5> masses_;

  Eigen::Matrix3d roll_rotation_;
  Eigen::Matrix3d pitch_rotation_;

  Eigen::Matrix4d pose_;

  Eigen::Matrix<double, 3, 5> coms_;

  Eigen::Matrix<double, NumDoFs, 1> current_position_;
  Eigen::Matrix<double, NumDoFs, 1> current_position_command_;
  Eigen::Matrix<float, NumDoFs, 1> current_velocity_;
  Eigen::Matrix<float, NumDoFs, 1> current_velocity_command_;
  Eigen::Matrix<float, NumDoFs, 1> current_velocity_error_;
  Eigen::Matrix<double, 3, NumDoFs> current_gyros_;
  Eigen::Matrix<float, 4, NumDoFs> current_orientation_;
  Eigen::Matrix<double, 3, NumDoFs> rpy_modules_;

  alignas(64) std::array<Eigen::Matrix4d, 14> imu_frames_;

  //----------------------------------------------------------------------------
  // Velocity controller fields
  Chassis::VelocityControllerParams vc_params_;

  void update_com();
  void update_pose_estimate();
  void calculate_lean_angle();

  void soft_startup();
  void spin_once(bool bc);

  void stop_controller();
  void start_controller();

  void perform_start(std::condition_variable& start_condition);

public:

  Igor();

  void start();
  void request_stop();
  void set_balance_controller_state(bool enabled);
  void wait_for();

//------------------------------------------------------------------------------
// Accessors

  bool started() {
    std::lock_guard<std::mutex> lock(state_lock_);
    return started_;
  }

  double joystick_dead_zone() const {
    return joystick_dead_zone_;
  }

  double wheel_radius() const {
    return wheel_radius_;
  }

  double wheel_base() const {
    return wheel_base_;
  }

  std::shared_ptr<util::Joystick> joystick() {
    return joystick_;
  }

  Chassis& chassis() {
    return chassis_;
  }

  Leg<false>& left_leg() {
    return left_leg_;
  }

  Leg<true>& right_leg() {
    return right_leg_;
  }

  Arm<false>& left_arm() {
    return left_arm_;
  }

  Arm<true>& right_arm() {
    return right_arm_;
  }

};

}
