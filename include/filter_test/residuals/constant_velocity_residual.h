/*
 * constant_velocity_residual.h
 *
 *  Created on: 12.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_VELOCITY_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_VELOCITY_RESIDUAL_H_

#include "filter_test/block_impl.h"
#include "filter_test/residual.h"

namespace tsif {

// This residual implements the following set of equations
// r_x = ( x_k + dt * v_k) - x_kp1;
// r_v = ( v_k) - v_kp1;

// We assume that the residuals are embedded in a vector space (i.e. tangent
// space for manifolds).
class ConstantVelocityResidual : public ResidualBase {
  enum BlockIndex { kPositionBlock = 0, kVelocityBlock };
  static const int kResidualPositionOffset = 0;
  static const int kResidualVelocityOffset = 3;

  struct InternalState {
    InternalState(const VectorOfBlocks& state)
        : p(state[kPositionBlock]->getValue<VectorBlock<3>>()),
          v(state[kVelocityBlock]->getValue<VectorBlock<3>>()) {}
    InternalState(Vector3& p1, Vector3& v1) : p(p1), v(v1) {}
    Vector3& p;
    Vector3& v;
  };

  static const bool kIsMergeable = true;
  static const int kResidualDimension = 6;
  static const int kResidualMinimalDimension = 6;

 public:
  ConstantVelocityResidual(
      const double position_sigma, const double velocity_sigma)
      : ResidualBase(
            kResidualDimension, kResidualMinimalDimension, kIsMergeable) {
    sqrt_information_position_ = 1 / position_sigma;
    sqrt_information_velocity_ = 1 / velocity_sigma;
  }

  ~ConstantVelocityResidual() {}

  //  virtual bool prepareResidual(const int64_t t1_ns, const int64_t t2_ns) {
  //    // this residual has nothing to prepare.
  //    return true;
  //  }

  virtual bool predict(
      const VectorOfBlocks& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorOfBlocks* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) {
    CHECK_NOTNULL(predicted_state);
    const double dt = kNanoSecondsToSeconds * (t2_ns - t1_ns);

    const Vector3& p_k = state[kPositionBlock]->getValue<VectorBlock<3>>();
    const Vector3& v_k = state[kVelocityBlock]->getValue<VectorBlock<3>>();
    Vector3& p_kp1 =
        (*predicted_state)[kPositionBlock]->getValue<VectorBlock<3>>();
    Vector3& v_kp1 =
        (*predicted_state)[kVelocityBlock]->getValue<VectorBlock<3>>();

    p_kp1 = p_k + dt * v_k;
    v_kp1 = v_k;

    // For vector spaces this is identical to the one in evaluate().
    if (jacobian_wrt_state1 != NULL) {
      // d(r(...))/d(p_k)
      (*jacobian_wrt_state1)[kPositionBlock].template block<3, 3>(
          kResidualPositionOffset, 0) = Matrix3::Identity();
      (*jacobian_wrt_state1)[kPositionBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) = Matrix3::Zero();

      // d(r(...))/d(v_k)
      (*jacobian_wrt_state1)[kVelocityBlock].template block<3, 3>(
          kResidualPositionOffset, 0) = dt * Matrix3::Identity();
      (*jacobian_wrt_state1)[kVelocityBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) = Matrix3::Identity();
    }

    return true;
  }

  virtual bool evaluate(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorXRef residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) {
    const Vector3& p_k = state1[kPositionBlock]->getValue<VectorBlock<3>>();
    const Vector3& v_k = state1[kVelocityBlock]->getValue<VectorBlock<3>>();
    const Vector3& p_kp1 = state2[kPositionBlock]->getValue<VectorBlock<3>>();
    const Vector3& v_kp1 = state2[kVelocityBlock]->getValue<VectorBlock<3>>();

    double dt = kNanoSecondsToSeconds * (t2_ns - t1_ns);
    residual.template block<3, 1>(0, 0) =
        sqrt_information_position_ * ((p_k + dt * v_k) - p_kp1);
    residual.template block<3, 1>(3, 0) =
        sqrt_information_velocity_ * ((v_k)-v_kp1);

    if (jacobian_wrt_state1 != NULL) {
      // d(r(...))/d(p_k)
      (*jacobian_wrt_state1)[kPositionBlock].template block<3, 3>(
          kResidualPositionOffset, 0) =
          sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state1)[kPositionBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) = Matrix3::Zero();

      // d(r(...))/d(v_k)
      (*jacobian_wrt_state1)[kVelocityBlock].template block<3, 3>(
          kResidualPositionOffset, 0) =
          sqrt_information_position_ * dt * Matrix3::Identity();
      (*jacobian_wrt_state1)[kVelocityBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) =
          sqrt_information_velocity_ * Matrix3::Identity();
    }

    if (jacobian_wrt_state2 != NULL) {
      // d(r(...))/d(p_kp1)
      (*jacobian_wrt_state2)[kPositionBlock].template block<3, 3>(
          kResidualPositionOffset, 0) =
          -sqrt_information_position_ * Matrix3::Identity();
      (*jacobian_wrt_state2)[kPositionBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) = Matrix3::Zero();

      // d(r(...))/d(v_kp1)
      (*jacobian_wrt_state2)[kVelocityBlock].template block<3, 3>(
          kResidualPositionOffset, 0) = Matrix3::Zero();
      (*jacobian_wrt_state2)[kVelocityBlock].template block<3, 3>(
          kResidualVelocityOffset, 0) =
          -sqrt_information_velocity_ * Matrix3::Identity();
    }

    return true;
  }

  virtual std::string getName() const {
    return "const velocity";
  }

  virtual bool inputTypesValid(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2) {
    bool all_types_ok = true;
    all_types_ok &=
        state1[kPositionBlock]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &=
        state1[kVelocityBlock]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &=
        state2[kPositionBlock]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &=
        state2[kVelocityBlock]->isBlockTypeCorrect<VectorBlock<3>>();
    TSIF_LOGEIF(
        !all_types_ok,
        "Constant velocity residual has wrong block types. Check your state "
        "indices!");
    return all_types_ok;
  }

  virtual bool checkJacobians(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2,
      const int64_t t1_ns, const int64_t t2_ns, const double delta) {
    const std::vector<const TimedMeasurementVector*>
        measurement_vectors;  // Create measurements.
    // Since residual is not depending on any measurements this is trivial :)
    return checkJacobiansImpl(
        state1, state2, measurement_vectors, t1_ns, t2_ns, delta);
  }

 private:
  double sqrt_information_position_;
  double sqrt_information_velocity_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_VELOCITY_RESIDUAL_H_ */
