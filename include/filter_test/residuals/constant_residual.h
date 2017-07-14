/*
 * constant_residual.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_RESIDUAL_H_

#include "filter_test/residual.h"

namespace tsif {

// We assume that the residuals are embedded in a vector space (i.e. tangent
// space for manifolds).
class ConstantResidual : public ResidualBase {
  static const bool kIsMergeable = true;
  static const int kResidualDimension = 3;
  static const int kResidualMinimalDimension = 3;

 public:
  ConstantResidual() : ResidualBase(kResidualDimension, kResidualMinimalDimension, kIsMergeable) {}

  ~ConstantResidual() {}

  //  virtual bool prepareResidual(const int64_t t1_ns, const int64_t t2_ns) {
  //    // this residual has nothing to prepare.
  //    return true;
  //  }

  virtual bool predict(
      const VectorOfBlocks& state,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns,
      VectorOfBlocks* predicted_state,
      std::vector<MatrixXRef>* jacobian_wrt_state1) {
    // TODO(burrimi): implement.
    CHECK(false) << "Not implemented yet!";  // TODO(burrimi): Implement.
    return false;
  }

  virtual bool evaluate(
      const VectorOfBlocks& state1,
      const VectorOfBlocks& state2,
      const std::vector<const TimedMeasurementVector*>& measurement_vectors,
      const int64_t t1_ns, const int64_t t2_ns, VectorXRef residual,
      std::vector<MatrixXRef>* jacobian_wrt_state1,
      std::vector<MatrixXRef>* jacobian_wrt_state2) {
    //CHECK(false) << "Not implemented yet!";  // TODO(burrimi): Implement.
    return false;
  }

  virtual std::string getName() const {
    return "const residual";
  }

  virtual bool inputTypesValid(
      const VectorOfBlocks& state1,
      const VectorOfBlocks& state2) {
    bool all_types_ok = true;
    all_types_ok &= state1[0]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &= state2[0]->isBlockTypeCorrect<VectorBlock<3>>();
    TSIF_LOGEIF(
        !all_types_ok,
        "Constant residual has wrong block types. Check your state indices!");
    return all_types_ok;
  }

  virtual bool checkJacobians(const VectorOfBlocks& state1,
                        const VectorOfBlocks& state2,const int64_t t1_ns, const int64_t t2_ns, const double delta) {
    // TODO(burrimi): implement.
    return true;
  }

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_RESIDUALS_CONSTANT_RESIDUAL_H_ */
