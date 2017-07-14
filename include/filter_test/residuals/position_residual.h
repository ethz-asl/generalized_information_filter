/*
 * position_residual.h
 *
 *  Created on: 12.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_RESIDUALS_POSITION_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_RESIDUALS_POSITION_RESIDUAL_H_

#include "filter_test/residual.h"

namespace tsif {

// We assume that the residuals are embedded in a vector space (i.e. tangent
// space for manifolds).
class PositionResidual : public ResidualBase {
  static const bool kIsMergeable = false;
  static const int kResidualDimension = 3;
  static const int kResidualMinimalDimension = 3;

 public:
  PositionResidual(const Matrix3& covariance)
      : ResidualBase(
            kResidualDimension, kResidualMinimalDimension, kIsMergeable) {
    // TODO(burrimi): should we use robust cholesky decomposition (ldlt) to also
    // handle semidefinite?
    Matrix3 L = covariance.llt()
                    .matrixL();  // Cholesky decomposition L*L^* = covariance
    sqrt_information_matrix_.setIdentity();
    L.triangularView<Eigen::Lower>().solveInPlace(sqrt_information_matrix_);
  }

  ~PositionResidual() {}

  //  virtual bool prepareResidual(const int64_t t1_ns, const int64_t t2_ns) {
  //    return true;
  //  }

  virtual bool predict(
      const VectorOfBlocks& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorOfBlocks* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) {
    // TODO(burrimi): implement.
    assert(true);  // TODO(burrimi): Implement.
    return false;
  }

  inline Vector3 getPositionMeasurement(
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t2_ns) {
    CHECK(!measurement_vectors.empty());

    const TimedMeasurementVector* position_measurements =
        measurement_vectors[0];
    CHECK(!position_measurements->empty());

    const TimedMeasurement& current_measurement = (*position_measurements)[0];
    CHECK(current_measurement.first == t2_ns);

    const PositionMeasurement* position_measurement =
        dynamic_cast<const PositionMeasurement*>(current_measurement.second);
    CHECK_NOTNULL(position_measurement);

    const Vector3 position_measured = position_measurement->position_;

    return position_measured;
  }

  virtual bool evaluate(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorXRef residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) {
    const Vector3 position_measured =
        getPositionMeasurement(measurement_vectors, t2_ns);
    //    const Vector3 position_measured(0,0,0);

    const Vector3& p_kp1 = state2[0]->getValue<VectorBlock<3>>();

    residual.template block<3, 1>(0, 0) =
        sqrt_information_matrix_ * (p_kp1 - position_measured);

    if (jacobian_wrt_state2 != NULL) {
      (*jacobian_wrt_state2)[0].template block<3, 3>(0, 0) =
          sqrt_information_matrix_ * Matrix3::Identity();
    }

    return true;
  }

  virtual std::string getName() const {
    return "Position residual";
  }

  virtual bool inputTypesValid(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2) {
    bool all_types_ok = true;
    all_types_ok &= state2[0]->isBlockTypeCorrect<VectorBlock<3>>();
    TSIF_LOGEIF(
        !all_types_ok,
        "Position residual has wrong block types. Check your state indices!");
    return all_types_ok;
  }

  virtual bool checkJacobians(
      const VectorOfBlocks& state1, const VectorOfBlocks& state2,
      const int64_t t1_ns, const int64_t t2_ns, const double delta) {
    // TODO(burrimi): Find better way to create measurements. This is really
    // bad.
    std::vector<const TimedMeasurementVector*> measurement_vectors(
        1);  // Create measurements.
    PositionMeasurement position_measurement =
        PositionMeasurement::createRandomMeasurement();
    TimedMeasurementVector position_measurements;
    position_measurements.emplace_back(
        TimedMeasurement(t2_ns, &position_measurement));
    measurement_vectors[0] = &position_measurements;

    return checkJacobiansImpl(
        state1, state2, measurement_vectors, t1_ns, t2_ns, delta);
  }

 private:
  Matrix3 sqrt_information_matrix_;
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUALS_POSITION_RESIDUAL_H_ */
