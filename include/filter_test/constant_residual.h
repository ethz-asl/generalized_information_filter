/*
 * constant_residual.h
 *
 *  Created on: 02.06.2017
 *      Author: burrimi
 */

#ifndef INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_
#define INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_

#include "filter_test/residual.h"

namespace tsif {

// We assume that the residuals are embedded in a vector space (i.e. tangent space for manifolds).
class ConstantResidual : public ResidualBase {
  static const bool kIsMergeable = true;
  static const int kResidualDimension = 3;

 public:
  ConstantResidual() : ResidualBase(kResidualDimension, kIsMergeable) {}

  ~ConstantResidual() {}

  virtual bool prepareResidual(const int t1_ns, const int t2_ns) {
    // this residual has nothing to prepare.
    return true;
  }

  virtual bool evaluate(const std::vector<BlockBase*>& state1, const std::vector<BlockBase*>& state2, const int t1_ns,
                        const int t2_ns, VectorXRef* residual, std::vector<MatrixXRef>* jacobian_wrt_state1,
                        std::vector<MatrixXRef>* jacobian_wrt_state2) {
    assert(true); // TODO(burrimi): Implement.
    return true;
  }

  virtual std::string getPrintableName() const { return "const residual"; }

  virtual bool inputTypesValid(const std::vector<BlockBase*>& state1, const std::vector<BlockBase*>& state2) {
    bool all_types_ok = true;
    all_types_ok &= state1[0]->isBlockTypeCorrect<VectorBlock<3>>();
    all_types_ok &= state2[0]->isBlockTypeCorrect<VectorBlock<3>>();
    TSIF_LOGEIF(!all_types_ok, "Constant residual has wrong block types. Check your state indices!");
    return all_types_ok;
  }

 private:
};

}  // namespace tsif

#endif /* INCLUDE_FILTER_TEST_CONSTANT_RESIDUAL_H_ */
