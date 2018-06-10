#pragma once

#include <Eigen/Eigen>

#include <cmath>
#include <type_traits>

namespace hebi {
namespace util {

template<typename S, size_t N> using SquareMatrix = Eigen::Matrix<S, N, N>;
template<typename S> using Matrix3 = Eigen::Matrix<S, 4, 4>;
template<typename S> using Matrix4 = Eigen::Matrix<S, 4, 4>;
template<typename S> using Vector4 = Eigen::Matrix<S, 4, 1>;
template<typename S> using Vector3 = Eigen::Matrix<S, 3, 1>;

namespace detail {

template<typename T, typename S>
void rotateX(T& matrix, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  const S c = std::cos(angle);
  const S s = std::sin(angle);
  matrix(1, 1) = c;
  matrix(2, 1) = s;
  matrix(1, 2) = -s;
  matrix(2, 2) = c;
}

template<typename T, typename S>
void rotateY(T& matrix, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  const S c = std::cos(angle);
  const S s = std::sin(angle);
  matrix(0, 0) = c;
  matrix(0, 2) = s;
  matrix(2, 0) = -s;
  matrix(2, 2) = c;
}

template<typename T, typename S>
void rotateZ(T& matrix, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  const S c = std::cos(angle);
  const S s = std::sin(angle);
  matrix(0, 0) = c;
  matrix(0, 1) = s;
  matrix(1, 0) = -s;
  matrix(1, 1) = c;
}

template<typename S>
struct numeric_constants {
  static constexpr S PI = static_cast<S>(3.14159265358979323846264338327950288419716939937510582097494459230781640);
  static constexpr S NEG_2PI = static_cast<S>(-2.0*PI);
};

}

template<typename S, size_t N=4>
void rotateX(SquareMatrix<S, N>& mat, S angle) {
  static_assert(N > 2, "");
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  detail::rotateX(mat, angle);
}

template<typename S>
void rotateX(S* mat, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  Eigen::Map<Eigen::Matrix<S, 4, 4, Eigen::RowMajor>> map(mat);
  detail::rotateX(map, angle);
}

template<typename S, size_t N=4>
void rotateY(SquareMatrix<S, N>& mat, S angle) {
  static_assert(N > 2, "");
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  detail::rotateY(mat, angle);
}

template<typename S>
void rotateY(S* mat, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  Eigen::Map<Eigen::Matrix<S, 4, 4, Eigen::RowMajor>> map(mat);
  detail::rotateY(map, angle);
}

template<typename S, size_t N=4>
void rotateZ(SquareMatrix<S, N>& mat, S angle) {
  static_assert(N > 2, "");
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  detail::rotateZ(mat, angle);
}

template<typename S>
void rotateZ(S* mat, S angle) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  Eigen::Map<Eigen::Matrix<S, 4, 4, Eigen::RowMajor>> map(mat);
  detail::rotateZ(map, angle);
}

template<typename S, int Rows, int Cols>
void quat2rot(const Vector4<S>& quaternion, Eigen::Matrix<S, Rows, Cols>& output) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  static_assert(Rows >= 3, "Output matrix must have at least 3 rows");
  static_assert(Cols >= 3, "Output matrix must have at least 3 columns");

  const S X = quaternion[1];
  const S Y = quaternion[2];
  const S Z = quaternion[3];
  const S W = quaternion[0];

  const S xx = X*X;
  const S xy = X*Y;
  const S xz = X*Z;
  const S xw = X*W;

  const S yy = Y*Y;
  const S yz = Y*Z;
  const S yw = Y*W;

  const S zz = Z*Z;
  const S zw = Z*W;

  output(0, 0) = 1.0-2.0*(yy+zz);
  output(0, 1) = 2.0*(xy-zw);
  output(0, 2) = 2.0*(xz+yw);

  output(1, 0) = 2.0*(xy+zw);
  output(1, 1) = 1.0-2.0*(xx+zz);
  output(1, 2) = 2.0*(yz-xw);

  output(2, 0) = 2.0*(xz-yw);
  output(2, 1) = 2.0*(yz+xw);
  output(2, 2) = 1.0-2.0*(xx+yy);
}

template<typename S, int Rows, int Cols>
void rot2ea(Eigen::Matrix<S, Rows, Cols>& R, Vector3<S>& output) {
  static_assert(std::is_floating_point<S>::value, "Matrix scalar type must be floating point");
  static_assert(Rows >= 3, "Output matrix must have at least 3 rows");
  static_assert(Cols >= 3, "Output matrix must have at least 3 columns");

  const float sy1 = R(0, 0);
  const float sy2 = R(1, 0);
  const float sy = std::sqrt(sy1*sy1+sy2*sy2);
  const bool singular = sy < 1e-6f;

  S x, y, z;

  if (singular) {
    x = atan2(R(2, 1), R(2, 2));
    y = atan2(-R(2, 0), sy);
    z = atan2(sy2, sy1);
  } else { /* unlikely */
    output[0] = std::numeric_limits<float>::quiet_NaN();
    output[1] = std::numeric_limits<float>::quiet_NaN();
    output[2] = std::numeric_limits<float>::quiet_NaN();
    return;
  }

  const S pi = detail::numeric_constants<S>::PI;
  const S n_pi2 = detail::numeric_constants<S>::NEG_2PI;

  if (x > pi) {
    x += n_pi2;
  }
  if (y > pi) {
    y += n_pi2;
  }
  if (z > pi) {
    z += n_pi2;
  }

  output[0] = x;
  output[1] = y;
  output[2] = z;
}

}
}
