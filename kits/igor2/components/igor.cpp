#include <thread>

#include "components/igor.h"
#include "components/joystick_interface.h"
#include "util/rotation.h"
#include "group_feedback.hpp"
#include "lookup.hpp"

namespace hebi {

static constexpr double US_TO_S = 1.0/1000000.0;
static constexpr double MILLI_TO_SEC = 0.001;

template<typename T>
static bool any_nan(const T& mat, size_t Col) {
  return (
    std::isnan(mat(0, Col)) ||
    std::isnan(mat(1, Col)) ||
    std::isnan(mat(2, Col)) ||
    std::isnan(mat(3, Col))
  );
}

template<size_t Index, size_t MatSize, size_t IdxSize,
         typename IdxType=size_t, typename S=double>
static double matrix_finite_mean(
  const Eigen::Matrix<S, 3, MatSize>& mat,
  const std::array<IdxType, IdxSize>& index) {
  
  static_assert(MatSize >= IdxSize, "MatSize must be greater than or equal to IdxSize");

  double mean = 0.0;
  size_t nonnan_count = 0;

  for (size_t i : index) {
    auto val = static_cast<double>(mat(Index, i));
    if (std::isfinite(val)) {
      mean += val;
      nonnan_count++;
    }
  }

  if (nonnan_count > 0) {
    return mean / static_cast<double>(nonnan_count);
  }

  return 0.0;
}

template<size_t RowIndex, size_t Rows, size_t Cols, size_t IdxSize,
         typename IdxType=size_t, typename S=double>
static double colwise_finite_mean(const Eigen::Matrix<S, Rows, Cols>& mat,
                                  const std::array<IdxType, IdxSize>& index) {
  static_assert(RowIndex < Rows, "");
  static_assert(IdxSize <= Cols, "");

  double mean = 0.0;
  size_t nonnan_count = 0;

  for (size_t i : index) {
    auto val = static_cast<double>(mat(RowIndex, i));
    if (std::isfinite(val)) {
      mean += val;
      nonnan_count++;
    }
  }

  if (nonnan_count > 0) {
    return mean / static_cast<double>(nonnan_count);
  }

  return 0.0;
}

static double get_diff_rx_time(GroupFeedback& fbk,
                               std::array<uint64_t, 14>& last_time,
                               std::array<uint64_t, 14>& diff_time) {
  // NOTE: We terribly assume that `last_time` times are actually less
  // than the current times. This is very bad because of
  // numeric underflow if we are wrong, or if memory becomes corrupted.
  // Get with the times!
  for (size_t i = 0; i < 14; i++) {
    const auto& rx_time_field = fbk[i].actuator().receiveTime();
    if (rx_time_field) {
      uint64_t val = rx_time_field.get();
      diff_time[i] = val - last_time[i];
      last_time[i] = val;
    } else {
      diff_time[i] = 0;
      last_time[i] = 0;
    }
  }

  uint64_t sum = 0;
  size_t numNonZero = 0;

  for (size_t i = 0; i < 14; i++) {
    auto diff = diff_time[i];
    if (diff > 0) {
      numNonZero++;
      sum += diff;
    }
  }

  if (numNonZero == 0) {
    // This is pretty bad. Should warn or somtehing
    // Just be hacky and return default dt of 0.01;
    return 0.01;
  }

  uint64_t micros = sum / numNonZero;
  uint64_t us_to_s_mod = micros % 1000000;
  auto secs = static_cast<double>(micros / 1000000);
  secs += static_cast<double>(us_to_s_mod) * US_TO_S;
  return secs;
}

template<typename T>
static T clip(T val, T min, T max) {
  return std::min(std::max(val, min), max);
}

// TEMP
static std::shared_ptr<Group> find_igor() {
#if 1
  Lookup lookup;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  return lookup.getGroupFromNames({"Igor II"},
    {"wheel1", "wheel2",
     "hip1", "knee1",
     "hip2", "knee2",
     "base1", "shoulder1", "elbow1", "wrist1",
     "base2", "shoulder2", "elbow2", "wrist2"});
#else
  return hebi::Group::createImitation(14);
#endif
}

//------------------------------------------------------------------------------
//

Igor::Igor() {
  com_.setZero();
  rpy_modules_.setZero();
  pose_.setIdentity();
  current_position_.setZero();
  current_position_command_.setZero();
  current_velocity_.setZero();
  current_velocity_command_.setZero();
  current_velocity_error_.setZero();
  current_gyros_.setZero();
  current_orientation_.setZero();
  line_com_.setZero();
  ground_point_.setZero();
  roll_rotation_.setIdentity();
  pitch_rotation_.setIdentity();
}

//------------------------------------------------------------------------------
// Private Functions

void Igor::update_com() {
  coms_.col(0) = left_leg_.com();
  coms_.col(1) = right_leg_.com();
  coms_.col(2) = left_arm_.com();
  coms_.col(3) = right_arm_.com();

  const double inv_mass = 1.0 / mass_;
  com_[0] = coms_.row(0).dot(masses_) * inv_mass;
  com_[1] = coms_.row(1).dot(masses_) * inv_mass;
  com_[2] = coms_.row(2).dot(masses_) * inv_mass;
}

void Igor::update_pose_estimate() {
  // TODO

  // leg endeffectors
  imu_frames_[0] = left_leg_.current_tip_fk();
  imu_frames_[1] = right_leg_.current_tip_fk();
  // hip link output frames
  imu_frames_[3] = left_leg_.current_com_frame_at<1>();
  imu_frames_[5] = right_leg_.current_com_frame_at<1>();
  // arm base bracket
  imu_frames_[7] = left_arm_.current_com_frame_at<1>();
  imu_frames_[11] = right_arm_.current_com_frame_at<1>();
  // arm shoulder link
  imu_frames_[8] = left_arm_.current_com_frame_at<3>();
  imu_frames_[12] = right_arm_.current_com_frame_at<3>();
  // arm elbow link
  imu_frames_[9] = left_arm_.current_com_frame_at<5>();
  imu_frames_[13] = right_arm_.current_com_frame_at<5>();

  Eigen::Matrix3d q_rot;
  Eigen::Vector3d ea;
  q_rot.setIdentity();

  // FIXME: document below
  for (size_t i = 0; i < 14; i++) {
    current_gyros_.col(i).applyOnTheLeft(imu_frames_[i].topLeftCorner<3, 3>());
    if (any_nan(current_orientation_, i)) {
      rpy_modules_.col(i) = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    } else {
      util::quat2rot<double, 3, 3>(current_orientation_.col(i).cast<double>(), q_rot);
      q_rot.applyOnTheRight(imu_frames_[i].topLeftCorner<3, 3>().transpose());
      util::rot2ea<double, 3, 3>(q_rot, ea);
      rpy_modules_.col(i) = ea;
    }
  }

}

void Igor::calculate_lean_angle() {

  // Update roll and pitch angles
  roll_angle_ = matrix_finite_mean<0, NumDoFs, NumImuModules>(rpy_modules_, imu_modules_);
  pitch_angle_ = matrix_finite_mean<1, NumDoFs, NumImuModules>(rpy_modules_, imu_modules_);

  // Rotate by the pitch angle about the Y axis,
  // followed by rotating by the roll angle about the X axis
  util::rotateX<double, 3>(roll_rotation_, roll_angle_);
  util::rotateY<double, 3>(pitch_rotation_, pitch_angle_);
  pose_.topLeftCorner<3, 3>() = pitch_rotation_ * roll_rotation_;

  // rad/s
  feedback_lean_angle_velocity_ = colwise_finite_mean<1, 3, NumDoFs, 6>(current_gyros_, imu_modules_);

  // Find the mean of the two legs' translation vectors at the endeffector
  ground_point_ =
    (left_leg_.current_tip_fk().topRightCorner<3, 1>() +
      right_leg_.current_tip_fk().topRightCorner<3, 1>()) * 0.5;

  // Get the vector starting from the "ground point"
  // and ending at the position of the current center of mass,
  // then rotate it according to the current pitch angle of Igor
  line_com_ = pitch_rotation_ * (com_ - ground_point_);
  height_com_ = line_com_.norm();

  // Using the line center of mass, find the feedback lean angle
  feedback_lean_angle_ = std::atan2(line_com_[0], line_com_[2]) * 180.0 / M_PI;

  // Update Igor's center of mass by applying the transforms in the following order:
  //  1) pitch rotation
  //  2) roll rotation
  com_.applyOnTheLeft(pose_.topLeftCorner<3, 3>());

  // Update CoM of legs based on current pose estimate from calculated lean angle
  left_leg_.current_com_frame_at<0>().applyOnTheLeft(pose_);
  left_leg_.current_com_frame_at<1>().applyOnTheLeft(pose_);
  left_leg_.current_com_frame_at<2>().applyOnTheLeft(pose_);
  left_leg_.current_com_frame_at<3>().applyOnTheLeft(pose_);
  right_leg_.current_com_frame_at<0>().applyOnTheLeft(pose_);
  right_leg_.current_com_frame_at<1>().applyOnTheLeft(pose_);
  right_leg_.current_com_frame_at<2>().applyOnTheLeft(pose_);
  right_leg_.current_com_frame_at<3>().applyOnTheLeft(pose_);
}

void Igor::soft_startup() {
  group_->sendFeedbackRequest();
  group_->getNextFeedback(group_feedback_);

  for (size_t i = 0; i < 14; i++) {
    auto& actuator = group_feedback_[i].actuator();
    auto& rxTime = actuator.receiveTime();
    auto& position = actuator.position();
    if (rxTime) {
      last_rx_time_[i] = rxTime.get();
    } else {
      last_rx_time_[i] = 0;
    }
    current_position_[i] = position.get();
  }

  using high_resolution_clock = std::chrono::high_resolution_clock;

  auto l_leg_t = left_leg_.create_home_trajectory(current_position_);
  auto r_leg_t = right_leg_.create_home_trajectory(current_position_);
  auto l_arm_t = left_arm_.create_home_trajectory(current_position_);
  auto r_arm_t = right_arm_.create_home_trajectory(current_position_);

  const double soft_start_scale = 1.0 / 3.0;
  double t = 0.0;
  double left_knee = 0.0;
  double right_knee = 0.0;

  Vector3d gravity = -pose_.col(3).segment<3>(0);
  Eigen::Matrix<double, 4, 1> grav_comp_efforts;
  Eigen::VectorXd pos(4);
  Eigen::VectorXd vel(4);

  chassis_.update_time();

  auto start_time = high_resolution_clock::now();

  while (t < 3.0) {
    const double soft_start = std::min(t*soft_start_scale, 1.0);

    // Left Arm
    left_arm_.get_grav_comp_efforts(current_position_, gravity, grav_comp_efforts);
    l_arm_t->getState(t, &pos, &vel, nullptr);
    grav_comp_efforts *= soft_start;

    size_t idx = 0;
    for (size_t i : left_arm_.group_indices()) {
      auto& actuator = group_command_[i].actuator();
      actuator.position().set(pos[idx]);
      actuator.velocity().set(static_cast<float>(vel[idx]));
      actuator.effort().set(static_cast<float>(grav_comp_efforts[idx]));
      idx++;
    }

    // Right Arm
    right_arm_.get_grav_comp_efforts(current_position_, gravity, grav_comp_efforts);
    r_arm_t->getState(t, &pos, &vel, nullptr);
    grav_comp_efforts *= soft_start;

    idx = 0;
    for (size_t i : right_arm_.group_indices()) {
      auto& actuator = group_command_[i].actuator();
      actuator.position().set(pos[idx]);
      actuator.velocity().set(static_cast<float>(vel[idx]));
      actuator.effort().set(static_cast<float>(grav_comp_efforts[idx]));
      idx++;
    }

    // Left Leg
    l_leg_t->getState(t, &pos, &vel, nullptr);

    idx = 0;
    for (size_t i : left_leg_.group_indices()) {
      auto& actuator = group_command_[i].actuator();
      actuator.position().set(pos[idx]);
      actuator.velocity().set(static_cast<float>(vel[idx]));
      idx++;
    }

    left_knee = pos[1];

    // Right Leg
    r_leg_t->getState(t, &pos, &vel, nullptr);

    idx = 0;
    for (size_t i : right_leg_.group_indices()) {
      auto& actuator = group_command_[i].actuator();
      actuator.position().set(pos[idx]);
      actuator.velocity().set(static_cast<float>(vel[idx]));
      idx++;
    }

    right_knee = pos[1];

    // Send command
    group_->sendCommand(group_command_);
    group_->getNextFeedback(group_feedback_);

    // Update current position
    for (size_t i = 0; i < 14; i++) {
      current_position_[i] = group_feedback_[i].actuator().position().get();
    }

    auto time = high_resolution_clock::now() - start_time;
    t = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(time).count()) * US_TO_S;
  }

  left_leg_.set_knee_angle(left_knee);
  right_leg_.set_knee_angle(right_knee);

  for (size_t i = 0; i < 14; i++) {
    auto& actuator = group_feedback_[i].actuator();
    auto& rxTime = actuator.receiveTime();
    if (rxTime) {
      last_rx_time_[i] = rxTime.get();
    } else {
      last_rx_time_[i] = 0;
    }
  }
}

void Igor::spin_once(bool bc) {
  group_->getNextFeedback(group_feedback_);

  using high_resolution_clock = std::chrono::high_resolution_clock;
  using millis = std::chrono::milliseconds;
  auto relative_time = high_resolution_clock::now() - start_time_;

  const double soft_start =
    std::min(static_cast<double>(std::chrono::duration_cast<millis>(relative_time).count())*MILLI_TO_SEC, 1.0);
  const double dt = get_diff_rx_time(group_feedback_,
    last_rx_time_, diff_rx_time_);

  Eigen::Vector3d gyro_tmp;
  Eigen::Vector4f orntn_tmp;
  for (size_t i = 0; i < NumDoFs; i++) {
    auto& feedback = group_feedback_[i];
    auto& actuator = feedback.actuator();
    auto& imu = feedback.imu();
    current_position_[i] = actuator.position().get();
    current_position_command_[i] = actuator.positionCommand().get();
    const float velocity = actuator.velocity().get();
    const float velocityCommand = actuator.velocityCommand().get();
    current_velocity_[i] = velocity;
    current_velocity_command_[i] = velocityCommand;
    current_velocity_error_[i] = velocityCommand - velocity;

    const auto gyro = imu.gyro().get();
    const auto orntn = imu.orientation().get();

    gyro_tmp[0] = static_cast<double>(gyro.getX());
    gyro_tmp[1] = static_cast<double>(gyro.getY());
    gyro_tmp[2] = static_cast<double>(gyro.getZ());
    orntn_tmp[0] = orntn.getW();
    orntn_tmp[1] = orntn.getX();
    orntn_tmp[2] = orntn.getY();
    orntn_tmp[3] = orntn.getZ();

    current_gyros_.col(i) = gyro_tmp;
    current_orientation_.col(i) = orntn_tmp;
  }

  // TODO: optionally parallelize this
  left_leg_.on_feedback_received(current_position_, current_position_command_, current_velocity_, current_velocity_error_);
  right_leg_.on_feedback_received(current_position_, current_position_command_, current_velocity_, current_velocity_error_);
  left_arm_.on_feedback_received(current_position_, current_position_command_, current_velocity_, current_velocity_error_);
  right_arm_.on_feedback_received(current_position_, current_position_command_, current_velocity_, current_velocity_error_);

  update_com();
  update_pose_estimate();
  calculate_lean_angle();

  // Critical section
  {
    std::lock_guard<std::mutex> lock(value_lock_);

    const double user_commanded_knee_velocity = left_leg_.user_commanded_knee_velocity();
    const Vector3d& user_commanded_grip_velocity = left_arm_.user_commanded_grip_velocity();

    chassis_.update_trajectory(user_commanded_knee_velocity, user_commanded_grip_velocity);
    chassis_.integrate_step(dt);

    const double calculated_knee_velocity = chassis_.calculated_knee_velocity();
    const Vector3d calculated_grip_velocity = chassis_.calculated_grip_velocity();

    left_leg_.integrate_step(dt, calculated_knee_velocity);
    right_leg_.integrate_step(dt, calculated_knee_velocity);
    left_arm_.integrate_step(dt, calculated_grip_velocity);
    right_arm_.integrate_step(dt, calculated_grip_velocity);

    // Populate velocity controller parameters
    vc_params_.left_wheel_velocity = current_velocity_[0];
    vc_params_.right_wheel_velocity = current_velocity_[1];
    vc_params_.height_com = height_com_;
    vc_params_.feedback_lean_angle = feedback_lean_angle_;
    vc_params_.feedback_lean_angle_velocity = feedback_lean_angle_velocity_;

    chassis_.update_velocity_controller(dt, vc_params_);

    if (bc) {
      const double lean_P = lean_controller_.P();
      const double lean_I = lean_controller_.I();
      const double lean_D = lean_controller_.D();

      auto& l_wheel = group_command_[0].actuator().effort();
      auto& r_wheel = group_command_[1].actuator().effort();

      double effort = (lean_P * chassis_.lean_angle_error()) +
        (lean_I * chassis_.lean_angle_error_cumulative()) +
        (lean_D * feedback_lean_angle_velocity_);
      
      if (soft_start < 1.0) {
        effort *= soft_start;
      }

      const auto set_effort = static_cast<float>(effort);
      l_wheel.set(set_effort);
      r_wheel.set(-set_effort);
    } else {
      group_command_[0].actuator().effort().clear();
      group_command_[1].actuator().effort().clear();
    }

    // --------------
    // Wheel Commands
    double l_wheel_vel = chassis_.calculated_yaw_velocity() + chassis_.velocity_feedforward();
    double r_wheel_vel = chassis_.calculated_yaw_velocity() - chassis_.velocity_feedforward();

    l_wheel_vel = clip(l_wheel_vel, -max_wheel_velocity_, max_wheel_velocity_);
    r_wheel_vel = clip(r_wheel_vel, -max_wheel_velocity_, max_wheel_velocity_);

    group_command_[0].actuator().velocity().set(static_cast<float>(l_wheel_vel));
    group_command_[1].actuator().velocity().set(static_cast<float>(r_wheel_vel));

    // ------------
    // Leg Commands
    left_leg_.update_command(group_command_, roll_angle_, soft_start);
    right_leg_.update_command(group_command_, roll_angle_, soft_start);

    // ------------
    // Arm Commands
    left_arm_.update_command(group_command_, pose_, soft_start);
    right_arm_.update_command(group_command_, pose_, soft_start);
  }

  group_->sendCommand(group_command_);
}

void Igor::stop_controller() {
  auto duration = stop_time_ - start_time_;
  double sec = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count())*MILLI_TO_SEC;
  double freq = static_cast<double>(static_cast<double>(num_spins_) / sec);
  finished_cv_.notify_all();
  printf("ran for %.3f seconds.\nAverage frequency: %.3f Hz\n", sec, freq);
}

void Igor::start_controller() {

  // ------------------------------------------------
  // Some of our computations were delayed until here
  coms_.col(4) = chassis_.com();

  masses_[0] = left_leg_.mass();
  masses_[1] = right_leg_.mass();
  masses_[2] = left_arm_.mass();
  masses_[3] = right_arm_.mass();
  masses_[4] = chassis_.mass();

  mass_ = masses_[0] + masses_[1] + masses_[2] + masses_[3] + masses_[4];

  // These frames are constant
  imu_frames_[2] = left_leg_.base_frame();
  imu_frames_[4] = right_leg_.base_frame();
  imu_frames_[6] = left_arm_.base_frame();
  imu_frames_[10] = right_arm_.base_frame();

  vc_params_.wheel_radius = wheel_radius_;
  vc_params_.robot_mass = mass_;

  // -------------------
  // Begin startup phase

  soft_startup();

  auto availableJoysticks = util::Joystick::available_joysticks();
  if (availableJoysticks.empty()) {
    throw std::runtime_error("No joysticks found");
  }
  joystick_ = availableJoysticks[0];
  printf("Joystick found: %s\n", joystick_->name().c_str());

  register_igor_event_handlers(*this);
  start_time_ = std::chrono::high_resolution_clock::now();

  state_lock_.lock();
  while(!quit_flag_) {
    bool bc = balance_controller_enabled_;
    state_lock_.unlock();

    spin_once(bc);
    num_spins_++;

    state_lock_.lock();
  }

  stop_time_ = std::chrono::high_resolution_clock::now();

  // Need to make sure to unlock here!
  state_lock_.unlock();
  stop_controller();

}

void Igor::perform_start(std::condition_variable& start_condition) {
  {
    std::unique_lock<std::mutex> lk(state_lock_);
    started_ = true;
  }
  start_condition.notify_all();
  start_controller();
}

//------------------------------------------------------------------------------
// Public Functions

void Igor::start() {
  std::unique_lock<std::mutex> lock(state_lock_);
  if (started_) {
    return;
  }

  group_ = find_igor();
  if (group_.get() == nullptr) {
    // bad - print or something
    return;
  }

  group_->setCommandLifetimeMs(300);
  group_->setFeedbackFrequencyHz(100.0);

  group_command_.readGains(gains_file_);
  group_->sendCommandWithAcknowledgement(group_command_);

  std::condition_variable start_condition;
  // std::thread constructor passes arguments by copy.
  // Since we need a ref to the cv, use std::ref
  std::thread proc_thread(&Igor::perform_start, this, std::ref(start_condition));
  proc_thread.detach();
  start_condition.wait(lock);
}

void Igor::request_stop() {
  std::lock_guard<std::mutex> lock(state_lock_);
  if (!started_) {
    return;
  }
  quit_flag_ = true;
}

void Igor::set_balance_controller_state(bool enabled) {
  std::lock_guard<std::mutex> lock(state_lock_);
  balance_controller_enabled_ = enabled;
}

void Igor::wait_for() {
  std::unique_lock<std::mutex> lock(state_lock_);
  if (!started_) {
    return;
  }
  finished_cv_.wait(lock);
}

}
