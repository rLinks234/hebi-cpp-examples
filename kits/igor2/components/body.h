#pragma once

#include <array>
#include <cstddef>
#include <mutex>

#include <Eigen/Eigen>

#include "robot_model.hpp"
#include "trajectory.hpp"

namespace hebi {

class PIDController {

private:

  double p_;
  double i_;
  double d_;

public:

  PIDController() = delete;
  PIDController(const PIDController&) = delete;
  PIDController(PIDController&&) = delete;
  PIDController(double p, double i, double d)
    : p_(p), i_(i), d_(d) {}

  inline double P() const { return p_; }
  inline double I() const { return i_; }
  inline double D() const { return d_; }

};

template <bool NegateDirection>
struct DirectionResolver;

template<> struct DirectionResolver<false> {
  static constexpr double Direction = 1.0;
};

template<> struct DirectionResolver<true> {
  static constexpr double Direction = -1.0;
};

class BaseBody {

private:

  double mass_;
  Eigen::Vector3d com_;
  std::mutex& lock_;

protected:

  void set_mass(double mass) {
    mass_ = mass;
  }

  void set_com(double x, double y, double z) {
    com_[0] = x;
    com_[1] = y;
    com_[2] = z;
  }

public:

  BaseBody() = delete;
  BaseBody(const BaseBody&) = delete;
  BaseBody(BaseBody&&) = delete;
  virtual ~BaseBody() = default;

  BaseBody(std::mutex& lock) : lock_(lock), mass_(0.0) {
    com_[0] = com_[1] = com_[2] = 0.0;
  }

  BaseBody(std::mutex& lock, double mass,
           double com_x, double com_y, double com_z)
    : lock_(lock), mass_(mass) {
      com_[0] = com_x;
      com_[1] = com_y;
      com_[2] = com_z;
  }

  double mass() const {
    return mass_;
  }

  const Eigen::Vector3d& com() const {
    return com_;
  }

  std::unique_lock<std::mutex> lock_guard() {
    return std::unique_lock<std::mutex>(lock_);
  }

};

template <size_t DoFCount, size_t OutputFrameCount, size_t CoMFrameCount>
class PeripheralBody : public BaseBody {

protected:

  template<typename S> using VectorDoF = Eigen::Matrix<S, DoFCount, 1>;
  template<typename S> using JacobianMatrix = Eigen::Matrix<S, 6, DoFCount>;
  using Vector6f = Eigen::Matrix<float, 6, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

private:

  std::array<size_t, DoFCount> group_indices_;

protected:

  hebi::robot_model::RobotModel robot_;

  std::array<Eigen::Matrix4d, OutputFrameCount> current_fk_;
  std::array<Eigen::Matrix4d, CoMFrameCount> current_coms_;
  
  alignas(16) VectorDoF<double> feedback_position_;
  alignas(16) VectorDoF<double> feedback_position_command_;
  alignas(16) VectorDoF<double> impedance_torque_;
  alignas(16) VectorDoF<double> home_angles_;
  alignas(16) VectorDoF<float> feedback_velocity_;
  alignas(16) VectorDoF<float> feedback_velocity_error_;
  
  alignas(16) Eigen::Vector3d xyz_error_;
  alignas(16) Vector6d position_error_;
  alignas(16) Vector6d impedance_error_;
  alignas(16) Vector6d velocity_error_;

  alignas(64) Eigen::Matrix4d current_tip_fk_;

  alignas(64) JacobianMatrix<double> current_jacobians_actual_;
  alignas(64) JacobianMatrix<double> current_jacobians_expected_;

  alignas(64) Eigen::Matrix<double, CoMFrameCount, 1> masses_;
  alignas(64) Eigen::Matrix<double, 3, CoMFrameCount> current_xyz_;

  /**
   * Called after `on_feedback_received`
   */
  virtual void update_position();

public:

  PeripheralBody() = delete;
  PeripheralBody(const PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>&) = delete;
  PeripheralBody(PeripheralBody<DoFCount, OutputFrameCount, CoMFrameCount>&&) = delete;
  virtual ~PeripheralBody() = default;

  PeripheralBody(std::mutex& lock, std::array<size_t, DoFCount> indices)
    : BaseBody(lock), group_indices_(indices) {
    // TODO
  }

//

  const std::array<size_t, DoFCount>& group_indices() const {
    return group_indices_;
  }

  const Eigen::Matrix4d& current_tip_fk() const {
    return current_tip_fk_;
  }

  Eigen::Matrix4d base_frame() const {
    return robot_.getBaseFrame();
  }

  template<size_t Index>
  Matrix4d& current_fk_frame_at() {
    static_assert(Index < CoMFrameCount, "");
    return current_coms_[Index];
  }

  template<size_t Index>
  Eigen::Matrix4d& current_com_frame_at() {
    static_assert(Index < OutputFrameCount, "");
    return current_fk_[Index];
  }

//

  template <typename P, typename V>
  void on_feedback_received(const P& position,
                            const P& position_command,
                            const V& velocity,
                            const V& velocity_error) {
    size_t idx = 0;
    for (size_t i : group_indices()) {
      feedback_position_[idx] = position[i];
      feedback_position_command_[idx] = position_command[i];
      feedback_velocity_[idx] = velocity[i];
      feedback_velocity_error_[idx] = velocity_error[i];
      idx++;
    }

    update_position();
  }

  std::shared_ptr<hebi::trajectory::Trajectory> create_home_trajectory(const Eigen::VectorXd& position,
                                          double duration=3.0);

  VectorXd get_grav_comp_efforts(const Eigen::VectorXd& positions,
                                 const Eigen::Vector3d& gravty);

};

}
