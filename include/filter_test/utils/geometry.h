/*
 * geometry.h
 *
 *  Created on: 16.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_UTILS_GEOMETRY_H_
#define INCLUDE_FILTER_TEST_UTILS_GEOMETRY_H_

#include "filter_test/defines.h"

namespace tsif {

namespace common {

template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& vector) {
  Eigen::Matrix<Scalar, 3, 3> skew_matrix;
  skew_matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
      vector(0), 0;
  return skew_matrix;
}
}  // namespace common

// Quaternions are implemented as
//@article{bloesch2016primer,
//  title={A Primer on the Differential Calculus of 3D Orientations},
//  author={Bloesch, Michael and Sommer, Hannes and Laidlow, Tristan and Burri,
//  Michael and Nuetzi, Gabriel and Fankhauser, P{\'e}ter and Bellicoso, Dario
//  and Gehring, Christian and Leutenegger, Stefan and Hutter, Marco and
//  others},
//  journal={arXiv preprint arXiv:1606.05285},
//  year={2016}
//}
namespace quaternion_helpers {

// Returns the jacobian of the exponential map of SO3 d exp(phi)/(d phi).
// In bloesch2016primer this is the gamma function.
template <typename Scalar>
inline Eigen::Matrix<Scalar, 3, 3> expMapJacobian(
    const Eigen::Matrix<Scalar, 3, 1>& phi) {
  const Scalar phi_squared_norm = phi.squaredNorm();

  if (phi_squared_norm < kQuaternionSmallAngleThreshold) {
    Eigen::Matrix<Scalar, 3, 3> gamma;
    gamma.setIdentity();
    gamma += 0.5 * common::skew(phi);
    return gamma;
  }
  const Scalar phi_norm = sqrt(phi_squared_norm);
  const Eigen::Matrix<Scalar, 3, 3> phi_skew(common::skew(phi));

  Eigen::Matrix<Scalar, 3, 3> exp_map_jacobian;
  exp_map_jacobian.setIdentity();
  exp_map_jacobian += ((1.0 - cos(phi_norm)) / phi_squared_norm) * phi_skew;
  const Scalar phi_cubed = (phi_norm * phi_squared_norm);
  exp_map_jacobian +=
      ((phi_norm - sin(phi_norm)) / phi_cubed) * phi_skew * phi_skew;
  return exp_map_jacobian;
}

inline Matrix3 expMapJacobian(const Vector3& phi) {
  return expMapJacobian<double>(phi);
}

template <typename Scalar>
inline Eigen::Quaternion<Scalar> expMap(
    const Eigen::Matrix<Scalar, 3, 1>& theta) {
  const Scalar theta_squared_norm = theta.squaredNorm();

  if (theta_squared_norm < kQuaternionSmallAngleThreshold) {
    Eigen::Quaternion<Scalar> q(
        1, theta(0) * 0.5, theta(1) * 0.5, theta(2) * 0.5);
    q.normalize();
    return q;
  }

  const Scalar theta_norm = sqrt(theta_squared_norm);
  const Eigen::Matrix<Scalar, 3, 1> q_imag =
      sin(theta_norm * 0.5) * theta / theta_norm;
  Eigen::Quaternion<Scalar> q(
      cos(theta_norm * 0.5), q_imag(0), q_imag(1), q_imag(2));
  return q;
}

inline Quaternion expMap(const Vector3& theta) {
  return expMap<double>(theta);
}

inline Vector3 logMap(const Quaternion& q) {
  const Eigen::Block<const Eigen::Vector4d, 3, 1> q_imag = q.vec();
  const double q_imag_squared_norm = q_imag.squaredNorm();

  if (q_imag_squared_norm < kQuaternionSmallAngleThreshold) {
    return 2 * copysign(1, q.w()) * q_imag;
  }

  const double q_imag_norm = sqrt(q_imag_squared_norm);
  Vector3 q_log = 2 * atan2(q_imag_norm, q.w()) * q_imag / q_imag_norm;
  return q_log;
}

// Rotates the quaternion p with the error theta on the tangent space:
// p_plus_theta = p boxplus theta
inline void boxPlus(
    const Quaternion& p, const Vector3& theta, Quaternion* p_plus_theta) {
  CHECK_NOTNULL(p_plus_theta);
  *p_plus_theta = expMap(theta) * p;
}

// Calculates the shortest connection respecting the manifold structure:
// theta = p boxminus q
// TODO(burrimi): Extend this function to also return Jacobians to reuse
// computation.
inline void boxMinus(
    const Quaternion& p, const Quaternion& q, Vector3* p_minus_q) {
  CHECK_NOTNULL(p_minus_q);
  *p_minus_q = logMap(p * q.inverse());
}

// Calculates the Jacobian of the boxminus operator w.r.t. the two orientations
// p and q to properly account for the manifold structure.
// Reminder: theta = p boxminus q
// TODO(burrimi): Modify to also allow Eigen::Map types.
// TODO(burrimi): Deprecate this function and extend boxMinus() to also
// return Jacobians to reuse computation.
inline void GetBoxMinusJacobians(
    const Quaternion& p, const Quaternion& q, Matrix3* J_boxminus_wrt_p,
    Matrix3* J_boxminus_wrt_q) {
  if (J_boxminus_wrt_p == NULL && J_boxminus_wrt_q == NULL) {
    return;  // Nothing to do.
  }

  Vector3 theta;
  boxMinus(p, q, &theta);

  const Matrix3 gamma_inverse = expMapJacobian(theta).inverse();

  if (J_boxminus_wrt_p != NULL) {
    *J_boxminus_wrt_p = gamma_inverse;
  }

  if (J_boxminus_wrt_q != NULL) {
    const Quaternion delta_orientation = p * q.inverse();
    *J_boxminus_wrt_q = -gamma_inverse * delta_orientation.toRotationMatrix();
  }
}

}  // namespace quaternion_helper

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_UTILS_GEOMETRY_H_ */
