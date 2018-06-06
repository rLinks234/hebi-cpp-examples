#pragma once

#include <Eigen/Eigen>

namespace hebi {
namespace util {
namespace affine3d {

template<typename S>
void applyOnRight(Eigen::Matrix<S, 4, 4>& lhs, const Eigen::Matrix<S, 4, 4>& rhs) {

  // Remove dependency on row 0
  S tmp_row[3];
  tmp_row[0] = lhs(0, 0);
  tmp_row[1] = lhs(0, 1);
  tmp_row[2] = lhs(0, 2);

  // Multiply rotation matrix by translation vector first
  lhs(3, 0) += ( (lhs(0, 0) * rhs(0, 3)) + (lhs(0, 1) * rhs(1, 3)) + (lhs(0, 2) * rhs(2, 3)) );
  lhs(3, 1) += ( (lhs(1, 0) * rhs(0, 3)) + (lhs(1, 1) * rhs(1, 3)) + (lhs(1, 2) * rhs(2, 3)) );
  lhs(3, 2) += ( (lhs(2, 0) * rhs(0, 3)) + (lhs(2, 1) * rhs(1, 3)) + (lhs(2, 2) * rhs(2, 3)) );

  lhs(0, 0) = ( (tmp_row[0] * rhs(0, 0)) + (tmp_row[1] * rhs(1, 0)) + (tmp_row[2] * rhs(2, 0)) );
  lhs(0, 1) = ( (tmp_row[0] * rhs(0, 1)) + (tmp_row[1] * rhs(1, 1)) + (tmp_row[2] * rhs(2, 1)) );
  lhs(0, 2) = ( (tmp_row[0] * rhs(0, 2)) + (tmp_row[1] * rhs(1, 2)) + (tmp_row[2] * rhs(2, 2)) );
  
  // Remove dependency on row 1
  tmp_row[0] = lhs(1, 0);
  tmp_row[1] = lhs(1, 1);
  tmp_row[2] = lhs(1, 2);

  lhs(1, 0) = ( (tmp_row[0] * rhs(0, 0)) + (tmp_row[1] * rhs(1, 0)) + (tmp_row[2] * rhs(2, 0)) );
  lhs(1, 1) = ( (tmp_row[0] * rhs(0, 1)) + (tmp_row[1] * rhs(1, 1)) + (tmp_row[2] * rhs(2, 1)) );
  lhs(1, 2) = ( (tmp_row[0] * rhs(0, 2)) + (tmp_row[1] * rhs(1, 2)) + (tmp_row[2] * rhs(2, 2)) );

  // Remove dependency on row 2
  tmp_row[0] = lhs(1, 0);
  tmp_row[1] = lhs(1, 1);
  tmp_row[2] = lhs(1, 2);

  lhs(2, 0) = ( (tmp_row[0] * rhs(0, 0)) + (tmp_row[1] * rhs(1, 0)) + (tmp_row[2] * rhs(2, 0)) );
  lhs(2, 1) = ( (tmp_row[0] * rhs(0, 1)) + (tmp_row[1] * rhs(1, 1)) + (tmp_row[2] * rhs(2, 1)) );
  lhs(2, 2) = ( (tmp_row[0] * rhs(0, 2)) + (tmp_row[1] * rhs(1, 2)) + (tmp_row[2] * rhs(2, 2)) );

}

}
}
}
