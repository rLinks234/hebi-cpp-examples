#include <thread>

#include "components/igor.h"
#include "util/rotation.h"
#include "group_feedback.hpp"
#include "lookup.hpp"

namespace hebi {

Igor::Igor() {
  com_.setZero();
  pose_gyros_.setZero();
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
  roll_rotation_.setZero();
  pitch_rotation_.setZero();
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

template<typename T>
static bool any_nan(const T& mat, size_t Row) {
  return (
    std::isnan(mat(Row, 0)) ||
    std::isnan(mat(Row, 1)) ||
    std::isnan(mat(Row, 2)) ||
    std::isnan(mat(Row, 3))
  );
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
    pose_gyros_.col(i).applyOnTheLeft(imu_frames_[i].topLeftCorner<3, 3>());
    if (any_nan(current_orientation_, i)) {
      rpy_modules_.col(i) = std::numeric_limits<double>::quiet_NaN();
    } else {
      util::quat2rot<double, 4, 4>(current_orientation_.col(i), q_rot);
      q_rot.applyOnRight(imu_frames_[i].topLeftCorner<3, 3>().transpose());
      util::rot2ea<double, 3, 3>(q_rot, ea);
      rpy_modules_.col(i) = ea;
    }
  }

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
    double val = static_cast<double>(mat(Index, i));
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
    double val = static_cast<double>(mat(RowIndex, i));
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

void Igor::calculate_lean_angle() {

  // Update roll and pitch angles
  roll_angle_ = matrix_finite_mean<0, NumDoFs, NumImuModules>(rpy_modules_, imu_modules_);
  pitch_angle_ = matrix_finite_mean<1, NumDoFs, NumImuModules>(rpy_modules_, imu_modules_);

  // Rotate by the pitch angle about the Y axis,
  // followed by rotating by the roll angle about the X axis
  util::rotateX(roll_rotation_, roll_angle_);
  util::rotateY(pitch_rotation_, pitch_angle_);
  pose_.topLeftCorner<3, 3>() = pitch_rotation_ * roll_rotation_;

  // rad/s
  feedback_lean_angle_velocity_ = colwise_finite_mean<1, 3, NumDoFs, 6>(pose_gyros_, imu_modules_);

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
  feedback_lean_angle_ = std::atan2(line_com_[0], line_com_[2]);

  // Update Igor's center of mass by applying the transforms in the following order:
  //  1) pitch rotation
  //  2) roll rotation
  pose_.topLeftCorner<3, 3>().applyOnTheRight(line_com_);

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
  // TODO
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

  // FIXME: FINISH IMPLEMENTING
}

template<typename T>
static T clip(T val, T min, T max) {
  return std::min(std::max(val, min), max);
}

static constexpr double MILLI_TO_SEC = 0.001;

void Igor::spin_once(bool bc) {
  group_->getNextFeedback(group_feedback_);

  using high_resolution_clock = std::chrono::high_resolution_clock;
  using millis = std::chrono::milliseconds;
  auto relative_time = high_resolution_clock::now() - start_time_;

  const double soft_start =
    std::min(static_cast<double>(std::chrono::duration_cast<millis>(relative_time).count())*MILLI_TO_SEC, 1.0);
  const double dt = get_diff_rx_time(group_feedback_,
    last_rx_time_, diff_rx_time_);

  for (size_t i = 0; i < NumDoFs; i++) {
    auto& feedback = group_feedback_[i];
    auto& actuator = feedback.actuator();
    auto& imu = feedback.imu();
    current_position_[i] = actuator.position().get();
    current_position_command_[i] = actuator.positionCommand().get();
    const double velocity = actuator.velocity().get();
    const double velocityCommand = actuator.velocityCommand().get();
    current_velocity_[i] = velocity;
    current_velocity_command_[i] = velocityCommand;
    current_velocity_error_[i] = velocityCommand - velocity;

    const auto gyro = imu.gyro().get();
    const auto orntn = imu.orientation().get();

    current_gyros_.col(i) =
      Eigen::Vector3f(gyro.getX(), gyro.getY(), gyro.getZ());
    current_orientation_.col(i) =
      Eigen::Vector4f(orntn.getW(), orntn.getX(), orntn.getY(), orntn.getZ());
  }

  // TODO: optionally parallelize this
  left_leg_.on_feedback_received(current_position_);
  right_leg_.on_feedback_received(current_position_command_);
  left_arm_.on_feedback_received(current_velocity_);
  right_arm_.on_feedback_received(current_velocity_error_);

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
    // TODO

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

      l_wheel.set(effort);
      r_wheel.set(-effort);
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

    group_command_[0].actuator().velocity().set(l_wheel_vel);
    group_command_[1].actuator().velocity().set(r_wheel_vel);

    // ------------
    // Leg Commands
    left_leg_.update_command(group_command_, roll_angle_, soft_start);
    right_leg_.update_command(group_command_, roll_angle_, soft_start);

    // ------------
    // Arm Commands
    left_arm_.update_command(group_command_, pose_, soft_start);
    right_arm_.update_command(group_command_, pose_, soft_start);
  }
}

void Igor::stop_controller() {
  // TODO
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

  // -------------------
  // Begin startup phase

  soft_startup();

  // TODO: register event handlers
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

// TEMP
static std::shared_ptr<Group> find_igor() {
  Lookup lookup;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  return lookup.getGroupFromNames({"Igor II"},
    {"wheel1", "wheel2",
     "hip1", "knee1",
     "hip2", "knee2",
     "base1", "shoulder1", "elbow1", "wrist1",
     "base2", "shoulder2", "elbow2", "wrist2"});
}

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

  // TODO: find joystick
  // TODO: load gains

  std::condition_variable start_condition;
  std::thread proc_thread(&Igor::perform_start, this, start_condition);
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

}